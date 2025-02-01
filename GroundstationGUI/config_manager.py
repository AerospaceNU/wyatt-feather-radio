import json
import os

class ConfigManager:
    def __init__(self, filename="config.json"):
        self.filename = filename
        self.config = {}
        self.load()

    def load(self):
        """Load configuration from file if it exists."""
        if os.path.exists(self.filename):
            try:
                with open(self.filename, "r") as f:
                    self.config = json.load(f)
            except (json.JSONDecodeError, IOError):
                print("Warning: Failed to load configuration. Using defaults.")

    def save(self):
        """Save the current configuration to file."""
        try:
            with open(self.filename, "w") as f:
                json.dump(self.config, f, indent=4)
        except IOError:
            print("Error: Could not save configuration.")

    def set(self, key, value):
        """Set a configuration value and save it."""
        self.config[key] = value
        self.save()

    def get(self, key, default=None):
        """Get a configuration value, returning default if not found."""
        return self.config.get(key, default)
