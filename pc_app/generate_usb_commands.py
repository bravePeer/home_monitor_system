import re
import sys

def convert_header_to_python(header_file, output_file):
    with open(header_file, "r", encoding="utf-8") as f:
        lines = f.readlines()

    python_code = []
    enum_mode = False
    enum_name = ""
    enum_values = []
    python_code.append(f"\nfrom enum import Enum\n")
    
    for line in lines:
        line = line.rstrip()
        
        if line.strip().startswith("//"):
            python_code.append("#" + line[2:])
        elif line.strip().startswith("/*"):
            python_code.append(f'"""{line.strip("/* ")}"""')
        elif match := re.match(r"#define\s+(\w+)\s+(.+)", line):
            name, value = match.groups()
            python_code.append(f"{name} = {value}")
        elif match := re.match(r"const uint8_t(\s*)(\w+)(\s*=\s*\w*)", line):
            python_code.append(f"{match.group(2)}{match.group(3)}")
        elif match := re.match(r"enum class\s+(\w+)\s*:\s*\w+\s*", line):
            enum_mode = True
            enum_name = match.group(1)
            enum_values = []
        elif enum_mode and (match := re.match(r"(\s*)(\w+)(\s*=\s*\w*)", line)):
            enum_values.append(f"    {match.group(2)}{match.group(3)}")
        elif enum_mode and "}" in line:
            python_code.append(f"class {enum_name}(Enum):")
            python_code.extend(enum_values if enum_values else ["    pass"])
            python_code.append("\n")
            enum_mode = False
    
    with open(output_file, "w", encoding="utf-8") as f:
        f.write("\n".join(python_code))

# if __name__ == "__main__":
#     if len(sys.argv) < 3:
#         print("Użycie: python3 convert.py input.h output.py")
#     else:
#         convert_header_to_python(sys.argv[1], sys.argv[2])

convert_header_to_python("../home_monitor_system_code/include/rp2040/usb/usb_const.h", "monitor_station/usb_commands.py")
# convert_header_to_python("../home_monitor_system_code/include/sensor/sensor_data.hpp", "sensor/sensor_data.py")
