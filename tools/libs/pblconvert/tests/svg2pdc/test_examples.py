import os
import shutil
import unittest
from filecmp import dircmp

from pblconvert.svg2pdc.pdc import convert_to_png, serialize_image
from pblconvert.svg2pdc.svg import surface_from_svg

RECORD_MODE = False


class RenderExamplesTest(unittest.TestCase):
    def all_examples(self):
        result = []
        directory = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "examples"
        )

        for root, dirs, files in os.walk(directory):
            for f in files:
                if (
                    f.endswith(".svg")
                    and not f.endswith(".annotated.svg")
                    and not root.endswith(".actual")
                    and not root.endswith(".expected")
                ):
                    svg_file = os.path.join(root, f)
                    result.append(svg_file)

        return result

    def test_examples(self):
        for e in self.all_examples():
            actual_dir = os.path.splitext(e)[0] + ".actual"
            expected_dir = os.path.splitext(e)[0] + ".expected"

            if os.path.exists(actual_dir):
                shutil.rmtree(actual_dir)
            os.mkdir(actual_dir)

            if RECORD_MODE and os.path.exists(expected_dir):
                shutil.rmtree(expected_dir)

            surface = surface_from_svg(url=e, approximate_bezier=True)
            commands = surface.pdc_commands
            pdci = serialize_image(commands, surface.size())
            pdc_path = os.path.join(actual_dir, "image.pdc")
            with open(pdc_path, "wb") as f:
                f.write(pdci)

            png_path = os.path.join(actual_dir, "image.png")
            with open(png_path, "wb") as f:
                png_data = convert_to_png(pdci)
                f.write(png_data)

            annotated_path = os.path.join(actual_dir, "annotated.svg")
            with open(annotated_path, "w") as f:
                et = surface.element_tree()
                et.write(f, pretty_print=True)

            annotated_png_path = os.path.splitext(annotated_path)[0] + ".png"
            with open(annotated_png_path, "wb") as f:
                annotated_png_data = surface.render_annoations_on_top(png_path)
                f.write(annotated_png_data)

            if not os.path.exists(expected_dir):
                os.rename(actual_dir, expected_dir)
            else:
                self.assertContainsSameFiles(actual_dir, expected_dir)
                # comparison passed, remove the actual dir so the output is manageable
                if os.path.exists(actual_dir):
                    shutil.rmtree(actual_dir)

    def assertContainsSameFiles(self, actual_dir, expected_dir):
        cmp = dircmp(actual_dir, expected_dir, ignore=[".DS_Store"])
        self.assertTrue(
            len(cmp.left_only) == 0,
            "'{}' missing in {}".format(",".join(cmp.left_only), cmp.right),
        )
        self.assertTrue(
            len(cmp.right_only) == 0,
            "'{}' unexpected in {}".format(",".join(cmp.right_only), cmp.left),
        )
        self.assertTrue(
            len(cmp.diff_files) == 0,
            "'{}' different between {} and {}".format(",".join(cmp.diff_files), cmp.left, cmp.right),
        )
        self.assertTrue(
            len(cmp.funny_files) == 0,
            "'{}' funny between {} and {}".format(",".join(cmp.funny_files), cmp.left, cmp.right),
        )


if __name__ == "__main__":
    unittest.main()
