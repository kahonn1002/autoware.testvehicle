"""Minimal reader for CarMaker InfoFiles (TestRun, Vehicle and IPGRoad files).

Only the TestRun-level keys are read here; road geometry is evaluated by the
IPGRoad API so that the converted map matches the simulator exactly.
"""

import re
from pathlib import Path

_NAMED_VALUE = re.compile(r"\$(\w+)(?:=(\S*))?")


class InfoFile:
    """Key/value view of an InfoFile with `$Name=default` named values resolved."""

    def __init__(self, entries, path=None, named_values=None):
        self.entries = entries
        self.path = path
        self.named_values = dict(named_values or {})

    @classmethod
    def read(cls, path, named_values=None):
        path = Path(path)
        return cls.parse(path.read_text(encoding="utf-8", errors="replace"), path, named_values)

    @classmethod
    def parse(cls, text, path=None, named_values=None):
        entries = {}
        block = None
        for line in text.splitlines():
            if block is not None and line.startswith("\t"):
                block.append(line[1:])
                continue
            block = None
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            if "=" in line:
                key, value = line.split("=", 1)
                entries[key.strip()] = value.strip()
            elif stripped.endswith(":"):
                block = entries[stripped[:-1].strip()] = []
        return cls(entries, path, named_values)

    def __contains__(self, key):
        return key in self.entries

    def resolve(self, value):
        """Replace `$Name=default` and `$Name` with overrides or their defaults."""
        def substitute(match):
            name, default = match.group(1), match.group(2)
            if name in self.named_values:
                return str(self.named_values[name])
            if default is None:
                raise ValueError(f"Named value ${name} has no default; pass --param {name}=VALUE")
            return default
        return _NAMED_VALUE.sub(substitute, value)

    def get(self, key, default=None):
        value = self.entries.get(key)
        if value is None or isinstance(value, list):
            return default
        return self.resolve(value)

    def lines(self, key):
        value = self.entries.get(key)
        if value is None:
            return []
        return [self.resolve(line) for line in (value if isinstance(value, list) else [value])]

    def floats(self, key, default=None):
        value = self.get(key)
        if not value:
            return default
        return [float(item) for item in value.split()]

    def number(self, key, default=None):
        values = self.floats(key)
        return values[0] if values else default

    def indices(self, prefix):
        """Return sorted N for keys of the form `<prefix>.N.<field>`."""
        pattern = re.compile(re.escape(prefix) + r"\.(\d+)\.")
        return sorted({int(match.group(1)) for key in self.entries
                       for match in [pattern.match(key)] if match})


def parse_named_values(pairs):
    """Turn repeated `NAME=VALUE` command-line arguments into a mapping."""
    values = {}
    for pair in pairs:
        name, separator, value = pair.partition("=")
        if not separator or not name.strip():
            raise ValueError(f"Expected NAME=VALUE, got {pair!r}")
        values[name.strip().lstrip("$")] = value.strip()
    return values
