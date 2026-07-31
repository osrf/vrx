# Draft reply to Ryan — BlueBoat meshes / rendering follow-up

Plain text for pasting into Gmail.

---

Hi Ryan,

We found the rendering problem, and it is a Gazebo quirk, not your model. Your
normal map is fine — it shows up and we confirmed it is actually contributing to
the shading.

The early-vs-late July difference turned out to be the clue. The early boat had
one texture (base colour only); the new one has three (albedo, normal, packed
metallic/roughness). Gazebo reads a simple base-colour material out of a .glb
fine, but does not reliably import a full PBR material embedded in the .glb —
hence the bright mis-lit facets.

The fix is on our side: we extract the maps and declare the material in SDF
instead of letting Gazebo read it from the file. Same mesh, unmodified, and it
renders correctly. Nothing changes in your export workflow — keep embedding
textures in the .glb. Your self-describing image names (Albedo-..., Normal-...,
Roughness-...) made this much easier, so please keep that up.

Three things from testing:

1. Scale. Side-Scan-Sonar measures 6.1 m and Surveyor 3.4 m, against a 1.19 m
   boat. Usually an unapplied object scale (Ctrl+A > Apply > Scale). But are
   they meant to be that large? If so we will just place them differently.

2. Materials. Propeller.glb and Thruster-propeller.glb have no material, so they
   render default grey. A base colour is plenty.

3. FYI only: COLLISION-Ping-Mount.glb is ~15x its own visual (3.7 m vs 0.24 m).
   No action needed, but it suggests one export in that batch went out at a
   different scale — worth watching in case it hits a visual mesh.

Your earlier questions:

Viewing distance — the boat is ~1.2 m and usually a few metres from the camera.
Maps do not need matching resolutions, just enough for the detail each carries:
your normal map earns 2048, the albedo is two flat colours so 1024 would look
the same, and the roughness map is a single solid colour so it could be a value
rather than an image. Not urgent.

For the four colours, four albedo maps sharing one normal and roughness would be
the easy version on our end.

Optional for future hulls: the bow currently points along -X, the reverse of our
convention. We compensate easily, so no need to redo anything — just handy to
flip next time.

Repo naming and layout as per my earlier links, and feel free to commit directly
to that branch.

Happy to jump on a call and show you what we see in Gazebo if that is easier.

All the best,
Brian
