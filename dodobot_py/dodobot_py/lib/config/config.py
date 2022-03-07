import os
import yaml


class Config:
    def __init__(self, config_name, base_dir):
        if not config_name.endswith(".yaml"):
            raise ValueError("Invalid config extension: {}. Must be .yaml".format(config_name))
        self.base_dir = os.path.expanduser(base_dir)

        self.config_path = os.path.join(self.base_dir, "config", config_name)
        self.config_dir = os.path.dirname(self.config_path)
        if not os.path.isdir(self.config_dir):
            os.makedirs(self.config_dir)

        self.load()

    def load(self):
        with open(self.config_path) as file:
            config = yaml.safe_load(file.read())
        self.__dict__.update(config)

    def to_dict(self):
        return {}
