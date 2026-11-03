# Model textures

These three original textures restore references in the flattened robot model.
They come from the upstream [description package](https://github.com/PR2/pr2_common/tree/9a8e4fbcf66523ca14faf651c23513ee1ac29ccb/pr2_description),
which declares BSD licensing in its package metadata. Exact source URLs and
SHA-256 digests are retained in [sources.json](sources.json).

The persisted model uses paths relative to its own directory. Fixture launchers
resolve those resources to file URIs at runtime, so no workstation location is
stored in the model. The automation fixture uses relative links into this
repository; move the sibling repositories together.
