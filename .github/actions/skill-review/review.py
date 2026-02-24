#!/usr/bin/env python3
"""
Skill-based PR code review via DigitalOcean Gradient Serverless Inference.

Reads skill definitions from .agents/skills/, fetches the PR diff from GitHub,
calls Claude via the Gradient endpoint, and posts inline review comments.
"""

import json
import os
import re
import sys
import time
from pathlib import Path

import openai
from github import Github

# ---------------------------------------------------------------------------
# Configuration from environment
# ---------------------------------------------------------------------------
GRADIENT_API_KEY = os.environ["GRADIENT_API_KEY"]
GITHUB_TOKEN = os.environ["GITHUB_TOKEN"]
SKILLS_PATH = os.environ.get("SKILLS_PATH", ".agents/skills")
PR_NUMBER = int(os.environ["PR_NUMBER"])
GITHUB_REPO = os.environ["GITHUB_REPO"]
MODEL = os.environ.get("MODEL", "claude-sonnet-4-6")
FAIL_ON_CRITICAL = os.environ.get("FAIL_ON_CRITICAL", "true").lower() == "true"

# Extensions worth reviewing in this Java/FRC repo
REVIEWABLE_EXTENSIONS = {
    ".java", ".kt", ".py", ".ts", ".tsx", ".js", ".jsx", ".go",
    ".rs", ".cpp", ".c", ".h", ".hpp",
}

# Files to skip regardless of extension
SKIP_PATTERNS = re.compile(
    r"(package-lock\.json|yarn\.lock|pnpm-lock\.yaml|\.lock$|"
    r"gradlew(\.bat)?$|\.gradle$|build/|dist/|generated/)",
    re.IGNORECASE,
)

# Max diff characters sent in a single API call (keep well under token limit)
MAX_DIFF_CHARS = 40_000

# Max number of API calls per review run; PRs that would exceed this are skipped
MAX_CHUNKS = 3

# ---------------------------------------------------------------------------
# Gradient / OpenAI-compatible client and shared GitHub client
# ---------------------------------------------------------------------------
client = openai.OpenAI(
    api_key=GRADIENT_API_KEY,
    base_url="https://inference.do-ai.run/v1",
)
gh_client = Github(GITHUB_TOKEN)


# ---------------------------------------------------------------------------
# Skill loading
# ---------------------------------------------------------------------------
def load_skills(skills_path: str) -> list[dict]:
    """
    Walk skills_path looking for SKILL.md files one level deep.
    Each skill directory is expected to contain a SKILL.md with optional
    YAML frontmatter including `name` and `applies-to` fields.

    Returns a list of dicts: {name, applies_to, content}
    """
    root = Path(skills_path)
    skills = []

    # Support both flat .md files and subdirectory SKILL.md layouts
    candidates = list(root.glob("*/SKILL.md")) + [
        p for p in root.glob("*.md") if p.name.lower() != "readme.md"
    ]

    for path in candidates:
        raw = path.read_text(encoding="utf-8")
        name, applies_to, body = _parse_skill(raw, path)
        skills.append({"name": name, "applies_to": applies_to, "content": body})
        print(f"  Loaded skill: {name} (applies-to: {applies_to or 'all'})")

    return skills


def _parse_skill(raw: str, path: Path) -> tuple[str, list[str] | None, str]:
    """Parse optional YAML frontmatter and return (name, applies_to, body)."""
    name = path.parent.name if path.name == "SKILL.md" else path.stem
    applies_to = None
    body = raw

    if raw.startswith("---"):
        end = raw.find("---", 3)
        if end != -1:
            frontmatter = raw[3:end].strip()
            body = raw[end + 3:].strip()
            for line in frontmatter.splitlines():
                if line.startswith("name:"):
                    name = line.split(":", 1)[1].strip()
                elif line.startswith("applies-to:"):
                    val = line.split(":", 1)[1].strip()
                    # Parse either YAML list or comma-separated string
                    if val.startswith("["):
                        applies_to = [x.strip().strip('"\'') for x in val.strip("[]").split(",") if x.strip()]
                    else:
                        applies_to = [x.strip() for x in val.split(",") if x.strip()]

    return name, applies_to, body


def select_skills_for_files(skills: list[dict], changed_extensions: set[str]) -> list[dict]:
    """Return skills that apply to at least one of the changed file extensions."""
    relevant = []
    for skill in skills:
        if skill["applies_to"] is None:
            relevant.append(skill)  # no filter → always include
        elif any(ext in skill["applies_to"] for ext in changed_extensions):
            relevant.append(skill)
    return relevant


def build_skills_block(skills: list[dict]) -> str:
    parts = []
    for skill in skills:
        parts.append(f"## Skill: {skill['name']}\n\n{skill['content']}")
    return "\n\n---\n\n".join(parts)


# ---------------------------------------------------------------------------
# PR diff fetching
# ---------------------------------------------------------------------------
def get_pr_files(repo_name: str, pr_number: int):
    """Fetch changed files from GitHub, filtered to reviewable extensions."""
    repo = gh_client.get_repo(repo_name)
    pr = repo.get_pull(pr_number)
    files = []
    for f in pr.get_files():
        if f.status == "removed":
            continue
        if SKIP_PATTERNS.search(f.filename):
            continue
        ext = Path(f.filename).suffix.lower()
        if ext not in REVIEWABLE_EXTENSIONS:
            continue
        if not f.patch:
            continue
        files.append({"filename": f.filename, "patch": f.patch, "status": f.status})
    return files


def chunk_files(files: list[dict], max_chars: int) -> list[list[dict]]:
    """Split file list into chunks that fit within max_chars when serialised."""
    chunks, current, current_len = [], [], 0
    for f in files:
        entry_len = len(f["filename"]) + len(f["patch"]) + 20
        if current and current_len + entry_len > max_chars:
            chunks.append(current)
            current, current_len = [], 0
        current.append(f)
        current_len += entry_len
    if current:
        chunks.append(current)
    return chunks


def format_diff_block(files: list[dict]) -> str:
    parts = []
    for f in files:
        parts.append(f"### {f['filename']} ({f['status']})\n```diff\n{f['patch']}\n```")
    return "\n\n".join(parts)


# ---------------------------------------------------------------------------
# Claude review
# ---------------------------------------------------------------------------
SYSTEM_PROMPT = (
    "You are a strict but fair code reviewer for an FRC (FIRST Robotics Competition) "
    "Java robot codebase. You review code exclusively against the provided skill "
    "guidelines — do not invent rules that aren't in the skills. Be precise and "
    "actionable. Only report issues you are confident about (>= 80% confidence)."
)

USER_PROMPT_TEMPLATE = """\
Review the following PR diff against the skill practices below.

<skills>
{skills_block}
</skills>

<diff>
{diff_block}
</diff>

Respond ONLY with valid JSON in this exact schema — no prose, no markdown fences:
{{
  "issues": [
    {{
      "file": "path/to/file.java",
      "position": 42,
      "severity": "critical|warning|suggestion",
      "skill": "name of the skill violated",
      "message": "what is wrong and how to fix it"
    }}
  ]
}}

Rules:
- "critical": code is incorrect, breaks a required pattern, or will cause a bug
- "warning": deviation from best practice that should be fixed
- "suggestion": minor style or improvement opportunity
- If no issues are found, return {{"issues": []}}
- "position" must be the 1-indexed line number within the patch string (counting the "@@ ... @@" hunk header as line 1); this is the value GitHub uses for inline diff comments
"""


def call_claude(skills_block: str, diff_block: str) -> list[dict]:
    prompt = USER_PROMPT_TEMPLATE.format(
        skills_block=skills_block,
        diff_block=diff_block,
    )
    response = client.chat.completions.create(
        model=MODEL,
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": prompt},
        ],
        temperature=0,
        max_tokens=4096,
    )
    raw = response.choices[0].message.content.strip()

    # Strip accidental markdown fences
    if raw.startswith("```"):
        raw = re.sub(r"^```[^\n]*\n?", "", raw)
        raw = re.sub(r"\n?```$", "", raw)

    try:
        data = json.loads(raw)
        return data.get("issues", [])
    except json.JSONDecodeError as e:
        print(f"  WARNING: Could not parse Claude response as JSON: {e}")
        print(f"  Raw response (first 500 chars): {raw[:500]}")
        return []


# ---------------------------------------------------------------------------
# Posting comments
# ---------------------------------------------------------------------------
def post_review(repo_name: str, pr_number: int, issues: list[dict]) -> None:
    """Post a PR review with inline comments and a summary."""
    if not issues:
        print("No issues found — skipping review post.")
        return

    repo = gh_client.get_repo(repo_name)
    pr = repo.get_pull(pr_number)

    # Build inline comments (GitHub requires position = 1-indexed line in the diff hunk)
    comments = []
    for issue in issues:
        body = f"**[{issue['severity'].upper()}]** `{issue['skill']}`\n\n{issue['message']}"
        comments.append({
            "path": issue["file"],
            "position": issue.get("position", 1),
            "body": body,
        })

    # Count by severity
    counts = {"critical": 0, "warning": 0, "suggestion": 0}
    for issue in issues:
        sev = issue.get("severity", "suggestion")
        counts[sev] = counts.get(sev, 0) + 1

    summary_lines = [
        "## Skill Review Results",
        "",
        f"| Severity | Count |",
        f"|----------|-------|",
        f"| 🔴 Critical | {counts['critical']} |",
        f"| 🟡 Warning | {counts['warning']} |",
        f"| 🔵 Suggestion | {counts['suggestion']} |",
        "",
    ]
    if counts["critical"] > 0:
        summary_lines.append("**Action required:** Critical issues must be resolved before merging.")
    summary = "\n".join(summary_lines)

    event = "REQUEST_CHANGES" if counts["critical"] > 0 else "COMMENT"

    try:
        pr.create_review(
            body=summary,
            event=event,
            comments=comments,
        )
        print(f"Posted review: {counts['critical']} critical, {counts['warning']} warnings, {counts['suggestion']} suggestions")
    except Exception as e:
        # Inline comments can fail if lines aren't in the diff; fall back to a plain comment
        print(f"  WARNING: Could not post inline review ({e}), falling back to summary comment")
        issue_list = "\n".join(
            f"- **{i['severity'].upper()}** `{i['file']}:{i.get('position','?')}` ({i['skill']}): {i['message']}"
            for i in issues
        )
        pr.create_issue_comment(f"{summary}\n### Issues\n{issue_list}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> int:
    print(f"Skill Review — repo: {GITHUB_REPO}, PR: #{PR_NUMBER}, model: {MODEL}")

    # 1. Load skills
    print(f"\nLoading skills from {SKILLS_PATH}...")
    skills = load_skills(SKILLS_PATH)
    if not skills:
        print("No skills found — nothing to review.")
        return 0

    # 2. Fetch PR diff
    print("\nFetching PR diff...")
    pr_files = get_pr_files(GITHUB_REPO, PR_NUMBER)
    if not pr_files:
        print("No reviewable files changed.")
        return 0
    print(f"  {len(pr_files)} reviewable file(s): {[f['filename'] for f in pr_files]}")

    # 3. Filter skills to relevant extensions
    changed_exts = {Path(f["filename"]).suffix.lower() for f in pr_files}
    relevant_skills = select_skills_for_files(skills, changed_exts)
    if not relevant_skills:
        print("No skills apply to the changed file types.")
        return 0
    print(f"  Using {len(relevant_skills)} skill(s): {[s['name'] for s in relevant_skills]}")
    skills_block = build_skills_block(relevant_skills)

    # 4. Review in chunks
    all_issues: list[dict] = []
    chunks = chunk_files(pr_files, MAX_DIFF_CHARS)
    if len(chunks) > MAX_CHUNKS:
        print(f"PR too large: {len(chunks)} chunks exceeds MAX_CHUNKS={MAX_CHUNKS}. Skipping review.")
        repo = gh_client.get_repo(GITHUB_REPO)
        pr = repo.get_pull(PR_NUMBER)
        pr.create_issue_comment(
            f"⚠️ **Skill review skipped** — this PR is too large to review automatically "
            f"({len(pr_files)} files, {len(chunks)} chunks > limit of {MAX_CHUNKS}). "
            f"Please request a manual review."
        )
        return 0
    print(f"\nReviewing {len(chunks)} chunk(s)...")
    for i, chunk in enumerate(chunks, 1):
        print(f"  Chunk {i}/{len(chunks)}: {[f['filename'] for f in chunk]}")
        diff_block = format_diff_block(chunk)
        issues = call_claude(skills_block, diff_block)
        print(f"    → {len(issues)} issue(s) found")
        all_issues.extend(issues)
        if i < len(chunks):
            time.sleep(1)  # be polite to the rate limiter

    # 5. Post review
    print(f"\nTotal issues: {len(all_issues)}")
    post_review(GITHUB_REPO, PR_NUMBER, all_issues)

    # 6. Exit code
    has_critical = any(i.get("severity") == "critical" for i in all_issues)
    if FAIL_ON_CRITICAL and has_critical:
        print("Exiting with code 1 — critical issues found.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
