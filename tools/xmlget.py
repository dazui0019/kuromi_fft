import xml.etree.ElementTree as ET
import json
import re
import os
import sys

settings_json_path = '.vscode/settings.json'
mdk_prj_path = 'MDK-ARM/mode2.uvprojx'

# 检查是否存在.uvprojx文件
if (len(mdk_prj_path) == 0):
    print("No project found.")
    sys.exit(0)

# 解析XML文件
tree = ET.ElementTree(file=mdk_prj_path)
root = tree.getroot()
IncludePath = root.find('.//TargetArmAds/Cads//IncludePath').text
Define = root.find('.//TargetArmAds/Cads//Define').text

# 获取 Defines 列表
if (Define != None):
    define_list = re.findall('[^, ]+', Define) # 这样提取出来后是一个list
else:
    define_list = []

# 获取 Include Path
if(IncludePath != None):
    raw_list = re.findall(r'(?:\.{2}|\\)[\\/]([^;]+)(?:;|$)', IncludePath) # 这样提取出来后是一个list
else:
    raw_list = []

compare_flag = False
path_list = []

# 排除重复的路径
for path in raw_list:
    for path_temp in path_list:
        if path == path_temp:
            compare_flag = True
    if compare_flag == True:
        compare_flag = False
    else:
        path_list.append(path)

# 如果定义了__RTTHREAD__，则修改path_list，在路径前面添加"..\"
if "__RTTHREAD__" in define_list:
    path_list = ["..\\" + path for path in path_list]
    path_list.append("board")
    path_list.append(".")

# 检查是否存在settings.json文件
if os.path.isfile(settings_json_path) != True:
    print("'settings.json' does not exist.")
    sys.exit(0)

with open(settings_json_path, "r", encoding='utf-8') as json_file:
    json_raw = json.load(json_file) # 将Json文件读取为Python对象

# 删除原来的 includePath 和 defines
if('C_Cpp.default.includePath' in json_raw):
    del json_raw['C_Cpp.default.includePath']
if('C_Cpp.default.defines') in json_raw:
    del json_raw['C_Cpp.default.defines']

# 重新添加 includePath 和 defines
json_raw['C_Cpp.default.includePath'] = path_list
json_raw['C_Cpp.default.defines'] = define_list

# 将修改保存到文件
with open(settings_json_path, "w", encoding='utf-8') as json_file:
    json.dump(json_raw, json_file, indent=4)

print("Done.")
