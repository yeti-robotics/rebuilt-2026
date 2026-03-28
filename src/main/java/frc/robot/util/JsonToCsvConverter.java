package frc.robot.util;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import com.fasterxml.jackson.dataformat.csv.CsvMapper;
import com.fasterxml.jackson.dataformat.csv.CsvSchema;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class JsonToCsvConverter {
    private final ObjectMapper jsonMapper;
    private final CsvMapper csvMapper;

    public JsonToCsvConverter() {
        this.jsonMapper = new ObjectMapper();
        this.csvMapper = new CsvMapper();
        this.csvMapper.configure(SerializationFeature.FAIL_ON_EMPTY_BEANS, false);
    }

    public void convertJsonToCsv(File jsonFile, File csvFile) throws IOException {
        JsonNode rootNode = jsonMapper.readTree(jsonFile);

        if (rootNode.isArray()) {
            convertJsonArrayToCsv(rootNode, csvFile);
        } else if (rootNode.isObject()) {
            convertJsonObjectToCsv(rootNode, csvFile);
        } else {
            throw new IllegalArgumentException("JSON must be either an object or array");
        }
    }

    private void convertJsonArrayToCsv(JsonNode arrayNode, File csvFile) throws IOException {
        if (arrayNode.isEmpty()) {
            throw new IllegalArgumentException("JSON array is empty");
        }

        JsonNode firstElement = arrayNode.get(0);
        if (!firstElement.isObject()) {
            throw new IllegalArgumentException("Array elements must be JSON objects");
        }

        CsvSchema.Builder schemaBuilder = CsvSchema.builder();
        List<String> fieldNames = new ArrayList<>();

        Iterator<String> fieldIterator = firstElement.fieldNames();
        while (fieldIterator.hasNext()) {
            String fieldName = fieldIterator.next();
            fieldNames.add(fieldName);
            schemaBuilder.addColumn(fieldName);
        }

        CsvSchema schema = schemaBuilder.build().withHeader();

        csvMapper.writerFor(JsonNode.class)
                .with(schema)
                .writeValue(csvFile, arrayNode);
    }

    private void convertJsonObjectToCsv(JsonNode objectNode, File csvFile) throws IOException {
        CsvSchema schema = CsvSchema.builder()
                .addColumn("key")
                .addColumn("value")
                .build()
                .withHeader();

        List<JsonNode> rows = new ArrayList<>();
        Iterator<String> fieldIterator = objectNode.fieldNames();

        while (fieldIterator.hasNext()) {
            String fieldName = fieldIterator.next();
            JsonNode fieldValue = objectNode.get(fieldName);

            SimpleKeyValueRow row = new SimpleKeyValueRow(fieldName, fieldValue.asText());
            rows.add(jsonMapper.valueToTree(row));
        }

        csvMapper.writerFor(JsonNode.class)
                .with(schema)
                .writeValue(csvFile, rows);
    }

    public void convertNestedJsonToCsv(File jsonFile, File csvFile, String... nestedKeys) throws IOException {
        JsonNode rootNode = jsonMapper.readTree(jsonFile);
        JsonNode targetNode = rootNode;

        for (String key : nestedKeys) {
            if (targetNode.has(key)) {
                targetNode = targetNode.get(key);
            } else {
                throw new IllegalArgumentException("Nested key '" + key + "' not found in JSON");
            }
        }

        if (!targetNode.isArray()) {
            throw new IllegalArgumentException("Target node must be an array for CSV conversion");
        }

        convertJsonArrayToCsv(targetNode, csvFile);
    }

    private static class SimpleKeyValueRow {
        private final String key;
        private final String value;

        public SimpleKeyValueRow(String key, String value) {
            this.key = key;
            this.value = value;
        }

        public String getKey() {
            return key;
        }

        public String getValue() {
            return value;
        }
    }
}
