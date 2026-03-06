#include <stddef.h>

const char g_hmi_embedded_schema_json[] =
    "{\n"
    "  \"system_menu_config\": {\n"
    "    \"version\": \"2.0.0\",\n"
    "    \"description\": \"Coffee sorter HMI schema subset for ESP32-S3\",\n"
    "    \"default_startup_profile\": \"operator_basic_ui\",\n"
    "    \"language_support\": {\n"
    "      \"default\": \"vi\",\n"
    "      \"available\": [\"vi\", \"en\", \"es\", \"zh\"],\n"
    "      \"fallback\": \"en\"\n"
    "    },\n"
    "    \"roles\": {\n"
    "      \"operator\": {\"level\": 10},\n"
    "      \"senior_operator\": {\"level\": 20},\n"
    "      \"qc_engineer\": {\"level\": 30},\n"
    "      \"maintenance_admin\": {\"level\": 40},\n"
    "      \"auditor\": {\"level\": 5}\n"
    "    },\n"
    "    \"hardware_communication\": {\n"
    "      \"fpga_nodes\": {\"count\": 4, \"protocol\": \"opc_ua\"},\n"
    "      \"stm32_plc\": {\"protocol\": \"modbus_rtu\", \"baud_rate\": 115200}\n"
    "    }\n"
    "  }\n"
    "}\n";

const size_t g_hmi_embedded_schema_json_length = sizeof(g_hmi_embedded_schema_json) - 1U;
