import bpy
import struct
import os
import math


def read_gi(context, filepath, collection):
    freq = int(math.sqrt(os.path.getsize(filepath)//12))
    with open(filepath, 'rb') as f:
        me = bpy.data.meshes.new('ImporedGIMesh')
        ob = bpy.data.objects.new('ImportedGI', me)
        ob.show_name = True
        # Link object to scene
        collection.objects.link(ob)

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

from bpy.props import StringProperty, CollectionProperty
from bpy.types import Operator, OperatorFileListElement


class ImportGI(Operator):
    bl_idname = "import_test.gi"
    bl_label = "Import geometry image"

    filename_ext = ""

    filter_glob: StringProperty(
            default='*',
            options={'HIDDEN'}
        )

    files: CollectionProperty(
            name='Files',
            type=OperatorFileListElement
        )

    directory: StringProperty(subtype='DIR_PATH')

    def invoke(self, context, _event):
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def execute(self, context):
        new_collection = bpy.data.collections.new('imported_gi')
        bpy.context.scene.collection.children.link(new_collection)
        for file in self.files:
            read_gi(context, os.path.join(self.directory, file.name), new_collection)
        return {'FINISHED'}


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
