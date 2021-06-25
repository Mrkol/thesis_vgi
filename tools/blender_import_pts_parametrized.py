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
        fp_size = struct.unpack('Q', f.read(8))[0]
        data = f.read(8 * 3 * fp_size)
        while data:
            # long double not supported
            fmt = 'ffffffff' if fp_size == 4 else 'dddddddd'
            floats = [struct.unpack(fmt, data[8*fp_size*i:8*fp_size*(i+1)]) for i in range(3)]
            x = len(vertices)
            triangles.append((x, x + 1, x + 2))
            vertices += [floats[i][0:3] for i in range(3)]
            uvs += [floats[i][6:8] for i in range(3)]

            data = f.read(8 * 3 * fp_size)

        me.from_pydata(vertices, [], triangles)
        me.update(calc_edges = True)

        uv_layer = me.uv_layers.new().data

        vert_loops = {}
        for loop in me.loops:
            vert_loops.setdefault(loop.vertex_index, []).append(loop.index)

        for i, coord in enumerate(uvs):
            # For every loop of a vertex
            for li in vert_loops[i]:
                uv_layer[li].uv = coord

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
    self.layout.operator(ImportPTS.bl_idname, text="Plain Triangle Soup")


def register():
    bpy.utils.register_class(ImportPTS)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)


def unregister():
    bpy.utils.unregister_class(ImportPTS)
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)


if __name__ == "__main__":
    register()
