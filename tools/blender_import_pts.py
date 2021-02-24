import bpy
import struct
import random


def read_pts(context, filepath):
    print("Importing plain triangle soup...")
    with open(filepath, 'rb') as f:
        me = bpy.data.meshes.new('ImporedPTSMesh')
        ob = bpy.data.objects.new('ImportedPTS', me)
        ob.show_name = True
        # Link object to scene
        new_collection = bpy.data.collections.new('new_collection')
        bpy.context.scene.collection.children.link(new_collection)
        new_collection.objects.link(ob)

        # 9 floats per vertex, 3 vertices, 4 bytes per float
        vertices = []
        uvs = []
        triangles = []
        data = f.read(9 * 3 * 4)
        while data:
            floats = [struct.unpack('fff', data[12*i:12*(i+1)]) for i in range(9)]
            x = len(vertices)
            triangles.append((x, x + 1, x + 2))
            vertices += [floats[3*i] for i in range(3)]
            uvs += [floats[2 + 3*i] for i in  range(3)]

            data = f.read(9 * 3 * 4)

        patch_count = int(uvs[0][1])

        random_colors = [[random.uniform(0, 1) for j in range(3)] for i in range(patch_count)]

        me.from_pydata(vertices, [], triangles)
        me.update(calc_edges = True)

        colors = me.vertex_colors.new()
        for i in range(len(vertices)):
            colors.data[i].color = random_colors[int(uvs[i][0])] + [0]

    # would normally load the data here

    return {'FINISHED'}


# ImportHelper is a helper class, defines filename and
# invoke() function which calls the file selector.
from bpy_extras.io_utils import ImportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty
from bpy.types import Operator


class ImportPTS(Operator, ImportHelper):
    """This appears in the tooltip of the operator and in the generated docs"""
    bl_idname = "import_test.pts"  # important since its how bpy.ops.import_test.some_data is constructed
    bl_label = "Import plain triangle soup"

    filter_glob: StringProperty(
        default="*",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )

    def execute(self, context):
        return read_pts(context, self.filepath)


# Only needed if you want to add into a dynamic menu
def menu_func_import(self, context):
    self.layout.operator(ImportPTS.bl_idname, text="Text Import Operator")


def register():
    bpy.utils.register_class(ImportPTS)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)


def unregister():
    bpy.utils.unregister_class(ImportPTS)
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)


if __name__ == "__main__":
    # register()

    # test call
    bpy.ops.import_test.pts('INVOKE_DEFAULT')
