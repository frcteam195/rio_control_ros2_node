import jinja2
import yaml
import pathlib
import argparse
from typing import List
parser = argparse.ArgumentParser(description='Render script')
parser.add_argument('--header_output', action="store", dest='header_file', default="")
parser.add_argument('--header_template', action="store", dest='header_template', default="")
parser.add_argument('--src_output', action="store", dest='src_file', default="")
parser.add_argument('--src_template', action="store", dest='src_template', default="")
args = parser.parse_args()

node_name = pathlib.Path(__file__).parent.parent.name
robot_name = ""

for item in pathlib.Path.cwd().parent.iterdir():
    if item.is_dir():
        if item.name.count("_Robot") > 0:
            robot_name = str(item.resolve())

if robot_name == "":
    raise Exception("Unable to determine robot project folder")

with open(robot_name + '/config/' + node_name + '.yaml', 'r') as stream:
    try:
        yaml_obj = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print("Error loading yaml file")
        raise exc


type_map = {
    None : "rclcpp::PARAMETER_NOT_SET",
    bool : "rclcpp::PARAMETER_BOOL",
    int : "rclcpp::PARAMETER_INTEGER",
    float : "rclcpp::PARAMETER_DOUBLE",
    str : "rclcpp::PARAMETER_STRING",
    bytes : "rclcpp::PARAMETER_BYTE_ARRAY",
    6 : "rclcpp::PARAMETER_BOOL_ARRAY",
    7 : "rclcpp::PARAMETER_INTEGER_ARRAY",
    8 : "rclcpp::PARAMETER_DOUBLE_ARRAY",
    9 : "rclcpp::PARAMETER_STRING_ARRAY",
}

lang_type_map = {
    None : "rclcpp::Parameter",
    bool : "bool",
    int : "int",
    float : "double",
    str : "std::string",
    bytes : "std::vector<char>",
    6 : "std::vector<bool>",
    7 : "std::vector<int>",
    8 : "std::vector<double>",
    9 : "std::vector<std::string>",
}

params = {}
lang_types = {}

for (k, v) in yaml_obj[node_name]["ros__parameters"].items():
    if type(v) is list:
        if all(isinstance(n, bool) for n in v):
            params[k] = type_map[6]
            lang_types[k] = lang_type_map[6]
        elif all(isinstance(n, int) for n in v):
            params[k] = type_map[7]
            lang_types[k] = lang_type_map[7]
        elif all(isinstance(n, float) for n in v):
            params[k] = type_map[8]
            lang_types[k] = lang_type_map[8]
        elif any(isinstance(n, float) for n in v):
            params[k] = type_map[8]
            lang_types[k] = lang_type_map[8]
        elif all(isinstance(n, str) for n in v):
            params[k] = type_map[9]
            lang_types[k] = lang_type_map[9]
        else:
            params[k] = type_map[None]
            lang_types[k] = lang_type_map[None]
        pass
    else:
        params[k] = type_map[type(v)]
        lang_types[k] = lang_type_map[type(v)]


# # Generate unit test template
env = jinja2.Environment(loader=jinja2.FileSystemLoader('/'), trim_blocks=True)
header_template = env.get_template(args.header_template)
header_result = header_template.render(params=params, node_name=node_name, lang_types=lang_types)
f = open(args.header_file, "w")
f.write(header_result)
f.close()

src_template = env.get_template(args.src_template)
src_result = src_template.render(params=params, node_name=node_name, lang_types=lang_types)
f = open(args.src_file, "w")
f.write(src_result)
f.close()