import bpy
import bmesh

# Assicurati di essere in Object Mode e seleziona l'oggetto cubo
bpy.ops.object.mode_set(mode='OBJECT')
obj = bpy.context.active_object

print("new_line")

# Assicurati che l'oggetto selezionato sia una mesh
if obj and obj.type == 'MESH':
    # Crea una rappresentazione BMesh dell'oggetto
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    
    # Ottieni la matrice di trasformazione dell'oggetto
    matrix_world = obj.matrix_world
    
    # Itera su tutti i vertici della mesh
    for vert in bm.verts:
        global_coord = matrix_world @ vert.co
        print(f"Vertex at {global_coord}")

    # Libera la memoria usata dal BMesh
    bm.free()
else:
    print("Seleziona un oggetto di tipo Mesh.")
