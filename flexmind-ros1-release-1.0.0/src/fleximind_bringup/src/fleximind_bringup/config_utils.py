import yaml


def load_config(config_file):
    """从YAML文件加载配置"""
    try:
        with open(config_file, "r") as file:
            config_data = yaml.safe_load(file)
    except Exception as e:
        config_data = {}

    return config_data


def save_config(config_file, config_data):
    """保存配置到YAML文件"""
    try:
        with open(config_file, "w") as file:
            yaml.dump(config_data, file, default_flow_style=False)
    except Exception as e:
        return False

    return True
