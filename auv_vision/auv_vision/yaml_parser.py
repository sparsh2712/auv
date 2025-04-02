import yaml
import os

def load_yaml_config(file_name):
    config_dir = os.path.join(os.path.dirname(__file__), '../config')
    file_path = os.path.join(config_dir, file_name)

    try:
        with open(file_path, 'r') as yaml_file:
            config = yaml.safe_load(yaml_file)
            return config
    except FileNotFoundError:
        print(f"Error: {file_name} not found in {config_dir}")
        return None
    except yaml.YAMLError as exc:
        print(f"Error parsing YAML file: {exc}")
        return None

print(load_yaml_config('vision_rs24_constants.yaml'))
