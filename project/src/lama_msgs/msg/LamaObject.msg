# id of lama object in the database, will be unique.
# When adding a vertex into the database, this id will be ignored.
int32 id

# id of lama object in the world (usage is implementation specific).
int32 id_in_world

# name of object
string name

# emitter id and name.
int32 emitter_id
string emitter_name

# type: "EDGE" or "VERTEX"
int32 type
int32 VERTEX = 1
int32 EDGE = 2

# for edges: start and end vertices
# for vertices: not used
int32[2] references

