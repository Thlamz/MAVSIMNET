import glob
import queue
import re
from typing import List, TypedDict

class Parameter(TypedDict):
    name: str
    type: str
    unit: str
    default: str
    description: str

def get_comments(end_line, lines):
    comment = ""
    if '//' in lines[end_line]:
        comment_line_index = end_line
        comment_lines = []
        while comment_line_index >= 0 and lines[comment_line_index].strip().startswith('//'):
            line: str = lines[comment_line_index]
            comment_lines = [line.strip().removeprefix('//')] + comment_lines
            comment_line_index -= 1

        
        comment = "\n".join(comment_lines)
    return comment
for filename in glob.iglob("../src/" + '**/*.ned', recursive=True):
    with open(filename, "r") as file:
        module_name = filename.removesuffix('.ned')[(filename.rfind('\\') or filename.rfind('/')) + 1:]
        lines = file.readlines()
        module_line = -1
        for index, line in enumerate(lines):
            if 'simple MAVLink' in line:
                module_line = index
                break
        
        if module_line == -1:
            continue

        module_description = get_comments(module_line - 1, lines)

        parameter_line = -1
        for index,line in enumerate(lines):
            if "parameters:" in line:
                parameter_line = index
                break
        
        parameters: List[Parameter] = []
        if parameter_line != -1:
            for index, line in enumerate(lines[parameter_line + 1:]):
                index += parameter_line + 1
                if 'submodules' in line or 'gates' in line:
                    break
                line = line.strip()
                if match := re.match(r'(volatile )?(\w+) (\w+)( @unit\((\w+)\))?( = (default\((.*)\))?)?;', line):
                    parameter = Parameter()
                    parameter['name'] = match.group(3)
                    if not parameter['name']:
                        continue
                    
                    parameter['type'] = match.group(2) or ""
                    parameter['unit'] = match.group(5) or ""
                    parameter['default'] = match.group(8) or ""
                    parameter['description'] = get_comments(index - 1, lines).replace('\n', '<br>')
                    parameters.append(parameter)

        documentation = f"# {module_name}\n" \
                        f"## Description\n" \
                        f"{module_description}\n" \
                        f"## Parameters\n\n" \
                        f"| Name | Type | Unit | Default value | Description |\n" \
                        f"| ---- | ---- | ---- | ------------- | ----------- |\n"

        for parameter in parameters:
            documentation += f"| {parameter['name']} | {parameter['type']} | {parameter['unit']} | {parameter['default']} | {parameter['description']} |\n"

        with open(f"./docs/Modules/{module_name}.md", "w+") as doc_file:
            doc_file.write(documentation)