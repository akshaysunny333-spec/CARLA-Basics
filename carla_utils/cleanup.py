# Cleanup utility for destroying game actors
def destroy_actors(actors):
    for actor in actors:
        if actor is not None:
            actor.destroy()
