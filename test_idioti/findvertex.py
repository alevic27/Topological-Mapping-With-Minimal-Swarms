import bpy
import bmesh

# Assicurati di essere in Object Mode e seleziona l'oggetto cubo
bpy.ops.object.mode_set(mode='OBJECT')
obj = bpy.context.active_object

# Assicurati che l'oggetto selezionato sia una mesh
if obj and obj.type == 'MESH':
    # Crea una rappresentazione BMesh dell'oggetto
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    
    # Itera su tutti gli spigoli della mesh
    for edge in bm.edges:
        v1, v2 = edge.verts
        print(f"Edge from {v1.co} to {v2.co}")

    # Libera la memoria usata dal BMesh
    bm.free()
else:
    print("Seleziona un oggetto di tipo Mesh.")
