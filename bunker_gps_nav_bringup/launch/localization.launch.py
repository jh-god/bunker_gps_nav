from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent))
from launch_helpers import generate


def generate_launch_description():
    return generate('localization')
