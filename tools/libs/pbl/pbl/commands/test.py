from pebble_tool.commands.base import BaseCommand


class TestCommand(BaseCommand):
    """Testing!"""

    command = "test"

    def __call__(self, *args):
        super().__call__(*args)
        print("Hi there!")
