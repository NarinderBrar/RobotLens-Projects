from pathlib import Path
import re
from urllib.parse import urlparse
from xml.sax.saxutils import quoteattr, unescape


RESOURCE = re.compile(r'(<(?:mesh|texture)\b[^>]*\bfilename=)([\"\x27])(.*?)\2')


def robot_description(urdf: Path) -> str:
    def resolve(match):
        value = unescape(match.group(3))
        if urlparse(value).scheme or Path(value).is_absolute():
            return match.group(0)
        resource = (urdf.parent / value).resolve()
        if not resource.is_file():
            raise FileNotFoundError(f"Robot description resource is missing: {resource}")
        return match.group(1) + quoteattr(resource.as_uri())

    return RESOURCE.sub(resolve, urdf.read_text(encoding="utf-8"))
