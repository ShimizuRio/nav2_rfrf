import json
import re
import os 

def json_load(str):
  # JSONファイルのパスを取得 
  package_path = os.path.join(os.path.dirname(__file__), 'config', f"{str}.json")
  # load commented json
  jsonfh=open(package_path, 'r')
  data=jsonfh.read()
  return json.loads(re.sub('//.*\n','',data))