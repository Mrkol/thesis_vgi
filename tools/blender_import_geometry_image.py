import bpy
import struct
import os
import math


def read_gi(context, filepath):
    freq = math.isqrt(os.path.getsize(filepath)//12)
    with open(filepath, 'rb') as f:
        me = bpy.data.meshes.new('ImporedGIMesh')
        ob = bpy.data.objects.new('ImportedGI', me)
        ob.show_name = True
        # Link object to scene
        new_collection = bpy.data.collections.new('new_collection')
        bpy.context.scene.collection.children.link(new_collection)
        new_collection.objects.link(ob)

        vertices = []
        triangles = []
        data = f.read(4 * 3 * freq*freq)
        for i in range(freq*freq):
            vertices.append(struct.unpack('fff', data[4*3*i:4*3*(i+1)]))

        for j in range(freq - 1):
            for i in range(freq - 1):
                triangles.append((freq*j + i, freq*j + i + 1, freq*j + freq + i + 1))
                triangles.append((freq*j + i, freq*j + freq + i + 1, freq*j + freq + i))

        me.from_pydata(vertices, [], triangles)
        me.update(calc_edges = True)

    return {'FINISHED'}


# ImportHelper is a helper class, defines filename and
# invoke() function which calls the file selector.
from bpy_extras.io_utils import ImportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty
from bpy.types import Operator


class ImportGI(Operator, ImportHelper):
    """This appears in the tooltip of the operator and in the generated docs"""
    bl_idname = "import_test.gi"  # important since its how bpy.ops.import_test.some_data is constructed
    bl_label = "Import geometry image"

    filter_glob: StringProperty(
        default="*",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )

    def execute(self, context):
        return read_gi(context, self.filepath)


# Only needed if you want to add into a dynamic menu
def menu_func_import(self, context):
    self.layout.operator(ImportGI.bl_idname, text="Geometry Image")


def register():
    bpy.utils.register_class(ImportGI)
    bpy.types.TOPBAR_MT_file_import.append(menu_func_import)


def unregister():
    bpy.utils.unregister_class(ImportGI)
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)


if __name__ == "__main__":
    register()
