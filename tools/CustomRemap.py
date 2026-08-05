"""Generate Fallout 4 FaceGen customization remap payloads.

Fallout loads the main head geometry and a companion ``_faceBones.nif`` with
the same vertex count.  FaceGen deforms the companion mesh, then uses binary
extra data on that mesh to reproduce the main mesh's skinning before copying
the result back.  The payload must be generated after export because vertex
bone indices refer to the serialized ``BSSkin::Instance::Bones`` order, not an
authoring application's vertex-group order.

``CustomizationRemapData`` contains one 12-byte record per vertex: four
half-float weights followed by four byte indices into the face-bones skin
palette.  ``CustomizationRemapNewBonesData`` contains 0xD0-byte records: a
zero-terminated name in a 0x80-byte field followed by the bone's 0x50-byte
in-memory transform.  Fallout resolves every named new bone in the live face
tree before extending the palette; an unresolved name rejects the remap.
Main-NIF-only influences must therefore collapse to a shared ancestor rather
than produce new-bone records for nodes absent from the live tree.

This post-export tool reads the required Fallout 4 NIF blocks directly using
only Python's standard library.
"""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import re
import struct
import tempfile
from typing import Sequence


REMAP_RECORD = struct.Struct("<4e4B")
NEW_BONE_NAME_SIZE = 0x80
NEW_BONE_RECORD_SIZE = 0xD0
NEW_BONE_TRANSFORM = struct.Struct("<20f")
NIF_VERSION = 0x14020007
NIF_NO_BLOCK = 0xFFFFFFFF
MAX_USABLE_BONE_INDEX = 0xFE  # Fallout treats 0xFF as an invalid sentinel.
FO4_USER_VERSION_2 = 130
SKINNED_VERTEX_FLAG = 1 << 6
SUPPORTED_SHAPE_TYPES = frozenset(
    {
        "BSTriShape",
        "BSSubIndexTriShape",
        "BSDynamicTriShape",
        "BSMeshLODTriShape",
        "BSLODTriShape",
    }
)


class FaceBonesError(RuntimeError):
    """An invalid input or unsupported NIF layout."""


@dataclass(frozen=True)
class NifHeader:
    data: bytes
    block_types: tuple[str, ...]
    block_type_indices: tuple[int, ...]
    block_sizes: tuple[int, ...]
    block_offsets: tuple[int, ...]
    strings: tuple[str, ...]
    user_version_2: int

    @classmethod
    def read(cls, path: Path) -> "NifHeader":
        data = path.read_bytes()
        try:
            offset = data.index(b"\n") + 1
        except ValueError as exc:
            raise FaceBonesError(f"'{path}' has no NIF header line") from exc

        def need(size: int) -> None:
            if offset + size > len(data):
                raise FaceBonesError(f"'{path}' has a truncated NIF header")

        def read_u8() -> int:
            nonlocal offset
            need(1)
            value = data[offset]
            offset += 1
            return value

        def read_u16() -> int:
            nonlocal offset
            need(2)
            value = struct.unpack_from("<H", data, offset)[0]
            offset += 2
            return value

        def read_u32() -> int:
            nonlocal offset
            need(4)
            value = struct.unpack_from("<I", data, offset)[0]
            offset += 4
            return value

        def read_short_string() -> bytes:
            nonlocal offset
            length = read_u8()
            need(length)
            value = data[offset : offset + length]
            offset += length
            return value

        def read_sized_string() -> bytes:
            nonlocal offset
            length = read_u32()
            need(length)
            value = data[offset : offset + length]
            offset += length
            return value

        version = read_u32()
        endian = read_u8()
        user_version = read_u32()
        block_count = read_u32()
        user_version_2 = read_u32()
        if (
            version != NIF_VERSION
            or endian != 1
            or user_version != 12
            or user_version_2 != FO4_USER_VERSION_2
        ):
            raise FaceBonesError(
                f"'{path}' is not a supported little-endian Fallout 4 NIF"
            )

        # Bethesda export metadata uses one-byte string lengths.
        for _ in range(3):
            read_short_string()
        if user_version_2 == 130:
            read_short_string()

        block_type_count = read_u16()
        block_types = tuple(
            read_sized_string().decode("utf-8", errors="strict")
            for _ in range(block_type_count)
        )
        block_type_indices = tuple(read_u16() for _ in range(block_count))
        block_sizes = tuple(read_u32() for _ in range(block_count))

        string_count = read_u32()
        read_u32()  # Maximum string length.
        strings = tuple(
            read_sized_string().decode("utf-8", errors="strict")
            for _ in range(string_count)
        )
        group_count = read_u32()
        for _ in range(group_count):
            read_u32()

        block_offsets: list[int] = []
        block_offset = offset
        for block_size in block_sizes:
            block_offsets.append(block_offset)
            block_offset += block_size
        if block_offset > len(data):
            raise FaceBonesError(f"'{path}' has truncated block data")

        for type_index in block_type_indices:
            if type_index >= len(block_types):
                raise FaceBonesError(f"'{path}' has an invalid block-type index")

        return cls(
            data=data,
            block_types=block_types,
            block_type_indices=block_type_indices,
            block_sizes=block_sizes,
            block_offsets=tuple(block_offsets),
            strings=strings,
            user_version_2=user_version_2,
        )

    def block_type(self, block_id: int) -> str:
        self._check_block_id(block_id)
        return self.block_types[self.block_type_indices[block_id]]

    def block(self, block_id: int) -> memoryview:
        self._check_block_id(block_id)
        offset = self.block_offsets[block_id]
        size = self.block_sizes[block_id]
        return memoryview(self.data)[offset : offset + size]

    def _check_block_id(self, block_id: int) -> None:
        if not 0 <= block_id < len(self.block_sizes):
            raise FaceBonesError(f"invalid NIF block id {block_id}")

    def node_name(self, block_id: int) -> str:
        block = self.block(block_id)
        if len(block) < 4:
            raise FaceBonesError(f"bone node block {block_id} is truncated")
        string_index = struct.unpack_from("<i", block, 0)[0]
        if not 0 <= string_index < len(self.strings):
            raise FaceBonesError(
                f"bone node block {block_id} has invalid name index {string_index}"
            )
        return self.strings[string_index]

    def skin_palette(self, skin_instance_id: int) -> tuple[list[str], list[int], int]:
        if self.block_type(skin_instance_id) != "BSSkin::Instance":
            raise FaceBonesError(
                f"block {skin_instance_id} is {self.block_type(skin_instance_id)}, "
                "not BSSkin::Instance"
            )
        block = self.block(skin_instance_id)
        if len(block) < 12:
            raise FaceBonesError("truncated BSSkin::Instance block")

        _skeleton_root, bone_data_id, bone_count = struct.unpack_from("<III", block, 0)
        required = 12 + bone_count * 4
        if len(block) < required:
            raise FaceBonesError("truncated BSSkin::Instance bone list")
        bone_ids = list(struct.unpack_from(f"<{bone_count}I", block, 12))
        names = [self.node_name(block_id) for block_id in bone_ids]
        return names, bone_ids, bone_data_id

    def bone_data(self, bone_data_id: int) -> list[tuple[float, ...]]:
        if self.block_type(bone_data_id) != "BSSkin::BoneData":
            raise FaceBonesError(
                f"block {bone_data_id} is {self.block_type(bone_data_id)}, "
                "not BSSkin::BoneData"
            )
        block = self.block(bone_data_id)
        if len(block) < 4:
            raise FaceBonesError("truncated BSSkin::BoneData block")
        count = struct.unpack_from("<I", block, 0)[0]
        expected = 4 + count * 68
        if len(block) != expected:
            raise FaceBonesError(
                f"unexpected BSSkin::BoneData size {len(block)} for {count} bones"
            )
        return [
            struct.unpack_from("<17f", block, 4 + index * 68)
            for index in range(count)
        ]


@dataclass(frozen=True)
class NifShape:
    block_id: int
    name: str
    skin_instance_id: int
    vertex_count: int
    vertex_desc: int
    weights: tuple[tuple[tuple[float, int], ...], ...]


@dataclass(frozen=True)
class NifDocument:
    header: NifHeader
    shapes: tuple[NifShape, ...]
    node_names: frozenset[str]
    parent_by_name: dict[str, str]
    root_names: frozenset[str]


@dataclass(frozen=True)
class SelectedShape:
    shape: NifShape
    palette: tuple[str, ...]
    bone_data: tuple[tuple[float, ...], ...]


@dataclass(frozen=True)
class GenerationResult:
    main_shape_name: str
    face_shape_name: str
    vertex_count: int
    main_bone_count: int
    face_bone_count: int
    appended_bones: tuple[str, ...]
    collapsed_bones: tuple[tuple[str, str], ...]
    remap_data: bytes
    new_bones_data: bytes


def _require_block_bytes(
    block: memoryview,
    offset: int,
    size: int,
    description: str,
) -> None:
    if offset < 0 or size < 0 or offset + size > len(block):
        raise FaceBonesError(f"truncated {description}")


def _object_net_end(header: NifHeader, block_id: int) -> int:
    block = header.block(block_id)
    _require_block_bytes(block, 0, 8, f"{header.block_type(block_id)} block")
    extra_count = struct.unpack_from("<I", block, 4)[0]
    end = 12 + extra_count * 4  # Name/count/list followed by Controller.
    _require_block_bytes(block, 0, end, f"NiObjectNET block {block_id}")
    return end


def _av_object_end(header: NifHeader, block_id: int) -> int:
    # Flags + translation + rotation + scale + collision object.
    end = _object_net_end(header, block_id) + 60
    _require_block_bytes(
        header.block(block_id), 0, end, f"NiAVObject block {block_id}"
    )
    return end


def _read_shape(header: NifHeader, block_id: int) -> NifShape:
    block_type = header.block_type(block_id)
    if block_type not in SUPPORTED_SHAPE_TYPES:
        raise FaceBonesError(f"unsupported geometry block type '{block_type}'")

    block = header.block(block_id)
    geometry_offset = _av_object_end(header, block_id)
    fixed_size = 46
    _require_block_bytes(
        block, geometry_offset, fixed_size, f"{block_type} block {block_id}"
    )
    (
        skin_instance_id,
        _shader_property_id,
        _alpha_property_id,
        vertex_desc,
        triangle_count,
        vertex_count,
        data_size,
    ) = struct.unpack_from("<IIIQIHI", block, geometry_offset + 16)

    stride = (vertex_desc & 0xF) * 4
    expected_size = vertex_count * stride + triangle_count * 6
    if data_size != expected_size:
        raise FaceBonesError(
            f"'{header.node_name(block_id)}' has data size {data_size}, "
            f"expected {expected_size} from its vertex descriptor"
        )
    vertex_data_offset = geometry_offset + fixed_size
    _require_block_bytes(
        block,
        vertex_data_offset,
        data_size,
        f"vertex data for '{header.node_name(block_id)}'",
    )

    flags = vertex_desc >> 44
    weights: list[tuple[tuple[float, int], ...]] = []
    if skin_instance_id != NIF_NO_BLOCK:
        if not flags & SKINNED_VERTEX_FLAG:
            raise FaceBonesError(
                f"'{header.node_name(block_id)}' has a skin instance but no "
                "skinning vertex attribute"
            )
        skin_offset = (vertex_desc >> 26) & 0x3C
        if stride == 0 or skin_offset + 12 > stride:
            raise FaceBonesError(
                f"'{header.node_name(block_id)}' has an invalid skinning offset"
            )
        for vertex_index in range(vertex_count):
            offset = vertex_data_offset + vertex_index * stride + skin_offset
            vertex_weights = struct.unpack_from("<4e", block, offset)
            bone_indices = struct.unpack_from("<4B", block, offset + 8)
            weights.append(
                tuple(
                    (float(weight), int(bone_index))
                    for weight, bone_index in zip(vertex_weights, bone_indices)
                    if weight > 0.0
                )
            )

    return NifShape(
        block_id=block_id,
        name=header.node_name(block_id),
        skin_instance_id=skin_instance_id,
        vertex_count=vertex_count,
        vertex_desc=vertex_desc,
        weights=tuple(weights),
    )


def _is_node_block(block_type: str) -> bool:
    # All node-derived types retain NiNode's child list immediately after the
    # NiAVObject fields.  Head assets normally use plain NiNode, but accepting
    # named node subclasses keeps the reader useful for exporter variations.
    return block_type in {"NiNode", "NiBone"} or block_type.endswith("Node")


def _read_nif(path: Path) -> NifDocument:
    header = NifHeader.read(path)
    shapes = tuple(
        _read_shape(header, block_id)
        for block_id in range(len(header.block_sizes))
        if header.block_type(block_id) in SUPPORTED_SHAPE_TYPES
    )

    node_ids = {
        block_id
        for block_id in range(len(header.block_sizes))
        if _is_node_block(header.block_type(block_id))
    }
    names_by_id = {block_id: header.node_name(block_id) for block_id in node_ids}
    parent_by_id: dict[int, int] = {}
    for parent_id in node_ids:
        block = header.block(parent_id)
        children_offset = _av_object_end(header, parent_id)
        _require_block_bytes(
            block, children_offset, 4, f"child list for node {parent_id}"
        )
        child_count = struct.unpack_from("<I", block, children_offset)[0]
        _require_block_bytes(
            block,
            children_offset + 4,
            child_count * 4,
            f"child list for node {parent_id}",
        )
        child_ids = struct.unpack_from(
            f"<{child_count}i", block, children_offset + 4
        ) if child_count else ()
        for child_id in child_ids:
            if child_id in node_ids:
                previous = parent_by_id.setdefault(child_id, parent_id)
                if previous != parent_id:
                    raise FaceBonesError(
                        f"node block {child_id} has more than one parent"
                    )

    parent_by_name: dict[str, str] = {}
    for child_id, parent_id in parent_by_id.items():
        child_name = names_by_id[child_id]
        parent_name = names_by_id[parent_id]
        previous = parent_by_name.setdefault(child_name, parent_name)
        if previous != parent_name:
            raise FaceBonesError(
                f"node name '{child_name}' has ambiguous parents"
            )

    root_names = frozenset(
        name for block_id, name in names_by_id.items() if block_id not in parent_by_id
    )
    return NifDocument(
        header=header,
        shapes=shapes,
        node_names=frozenset(names_by_id.values()),
        parent_by_name=parent_by_name,
        root_names=root_names,
    )


def _canonical_shape_name(name: str) -> str:
    value = name.casefold()
    value = re.sub(r"face[\s_.:-]*bones", "", value)
    value = re.sub(r"[\s_.:-]*\d+$", "", value)
    return re.sub(r"[^a-z0-9]+", "", value)


def _is_skinned(shape: NifShape) -> bool:
    return shape.skin_instance_id != NIF_NO_BLOCK and bool(shape.weights)


def _select_shape_pair(
    main_file: NifDocument,
    face_file: NifDocument,
) -> tuple[NifShape, NifShape]:
    main_shapes = [shape for shape in main_file.shapes if _is_skinned(shape)]
    face_shapes = [shape for shape in face_file.shapes if _is_skinned(shape)]
    if not main_shapes:
        raise FaceBonesError("the main NIF contains no skinned geometry")
    if not face_shapes:
        raise FaceBonesError("the face-bones NIF contains no skinned geometry")

    pairs = [
        (main_shape, face_shape)
        for main_shape in main_shapes
        for face_shape in face_shapes
        if main_shape.vertex_count == face_shape.vertex_count
    ]
    if not pairs:
        raise FaceBonesError(
            "no main/face-bones geometry pair has the same vertex count"
        )

    name_matches = [
        pair
        for pair in pairs
        if _canonical_shape_name(pair[0].name)
        == _canonical_shape_name(pair[1].name)
    ]
    if len(name_matches) == 1:
        return name_matches[0]
    if len(pairs) == 1:
        return pairs[0]

    candidates = ", ".join(
        f"'{main_shape.name}' -> '{face_shape.name}' "
        f"({main_shape.vertex_count} vertices)"
        for main_shape, face_shape in pairs
    )
    raise FaceBonesError(
        "multiple geometry pairs are possible; make their names unambiguous: "
        + candidates
    )


def _read_selected_shape(shape: NifShape, header: NifHeader) -> SelectedShape:
    palette, _bone_ids, bone_data_id = header.skin_palette(shape.skin_instance_id)
    bone_data = header.bone_data(bone_data_id)
    if len(palette) != len(bone_data):
        raise FaceBonesError(
            f"'{shape.name}' has {len(palette)} skin bones but "
            f"{len(bone_data)} bone-data records"
        )

    for vertex_index, row in enumerate(shape.weights):
        for _weight, bone_index in row:
            if bone_index >= len(palette):
                raise FaceBonesError(
                    f"'{shape.name}' vertex {vertex_index} references bone "
                    f"index {bone_index}, but its palette has {len(palette)} bones"
                )

    return SelectedShape(
        shape=shape,
        palette=tuple(palette),
        bone_data=tuple(bone_data),
    )


def _source_rows(source: SelectedShape) -> list[list[tuple[float, int]]]:
    rows = [list(row) for row in source.shape.weights]

    for vertex_index, row in enumerate(rows):
        if not row:
            raise FaceBonesError(
                f"main geometry vertex {vertex_index} has no skin weights"
            )
        if len(row) > 4:
            raise FaceBonesError(
                f"main geometry vertex {vertex_index} has {len(row)} influences; "
                "Fallout 4 remap records allow four"
            )
    return rows


def _disk_bone_transform_to_memory(record: Sequence[float]) -> bytes:
    if len(record) != 17:
        raise FaceBonesError("a BSSkin::BoneData record must contain 17 floats")
    values = (
        *record[0:4],
        record[4], record[7], record[10], 0.0,
        record[5], record[8], record[11], 0.0,
        record[6], record[9], record[12], 0.0,
        record[13], record[14], record[15], record[16],
    )
    return NEW_BONE_TRANSFORM.pack(*values)


def _is_live_tree_candidate(
    nif_file: NifDocument,
    name: str,
    face_node_names: set[str],
    destination_index: dict[str, int],
) -> bool:
    """Identify missing palette bones reachable from the live actor scaffold.

    Actor bones may be absent from the companion NIF but attach directly below
    its root or another shared bone.  Deeper missing branches are NIF-only.
    """
    if name in face_node_names:
        return True
    parent_name = nif_file.parent_by_name.get(name)
    return parent_name is not None and (
        parent_name in nif_file.root_names or parent_name in destination_index
    )


def _nearest_mapped_ancestor(
    nif_file: NifDocument,
    name: str,
    destination_index: dict[str, int],
) -> tuple[str, int]:
    current_name = name
    visited: set[str] = set()
    while current_name in nif_file.parent_by_name:
        if current_name in visited:
            raise FaceBonesError(f"cycle in source node ancestry for bone '{name}'")
        visited.add(current_name)
        ancestor_name = nif_file.parent_by_name[current_name]
        if ancestor_name in destination_index:
            return ancestor_name, destination_index[ancestor_name]
        current_name = ancestor_name
    raise FaceBonesError(
        f"bone '{name}' is absent from the live face-tree candidates and has "
        "no ancestor in the FaceBones skin palette"
    )


def generate_payloads(main_nif: os.PathLike[str] | str, face_nif: os.PathLike[str] | str) -> GenerationResult:
    main_path = Path(main_nif)
    face_path = Path(face_nif)
    if not main_path.is_file():
        raise FaceBonesError(f"main NIF does not exist: '{main_path}'")
    if not face_path.is_file():
        raise FaceBonesError(f"face-bones NIF does not exist: '{face_path}'")
    if main_path.resolve() == face_path.resolve():
        raise FaceBonesError("the main and face-bones NIF paths must be different")

    main_file = _read_nif(main_path)
    face_file = _read_nif(face_path)

    main_shape, face_shape = _select_shape_pair(main_file, face_file)
    main = _read_selected_shape(main_shape, main_file.header)
    face = _read_selected_shape(face_shape, face_file.header)
    if not any(name.casefold().startswith("skin_") for name in face.palette):
        raise FaceBonesError(
            f"'{face_shape.name}' does not contain any skin_ face bones"
        )

    destination_index: dict[str, int] = {}
    for index, name in enumerate(face.palette):
        destination_index.setdefault(name, index)

    face_node_names = set(face_file.node_names)
    source_to_destination: list[int | None] = [None] * len(main.palette)
    appended: list[tuple[str, int]] = []
    for source_index, name in enumerate(main.palette):
        target_index = destination_index.get(name)
        if target_index is None and _is_live_tree_candidate(
            main_file, name, face_node_names, destination_index
        ):
            target_index = len(face.palette) + len(appended)
            if target_index > MAX_USABLE_BONE_INDEX:
                raise FaceBonesError(
                    "the extended face-bones palette would use reserved index 0xFF"
                )
            destination_index[name] = target_index
            appended.append((name, source_index))
        source_to_destination[source_index] = target_index

    collapsed: list[tuple[str, str]] = []
    for source_index, name in enumerate(main.palette):
        if source_to_destination[source_index] is not None:
            continue
        ancestor_name, target_index = _nearest_mapped_ancestor(
            main_file, name, destination_index
        )
        source_to_destination[source_index] = target_index
        collapsed.append((name, ancestor_name))

    # Every optional slot is resolved above or raises with its bone name.
    resolved_destination = [
        int(index) for index in source_to_destination if index is not None
    ]
    if len(resolved_destination) != len(main.palette):
        raise FaceBonesError("internal error: unresolved source bone index")

    destination_names = {
        index: name for name, index in destination_index.items()
    }

    rows = _source_rows(main)
    remap_data = bytearray()
    for row in rows:
        # Several source bones may collapse to the same destination ancestor.
        # Merge those weights so each destination palette index occurs once.
        merged: dict[int, float] = {}
        for weight, source_index in row:
            target_index = resolved_destination[source_index]
            merged[target_index] = merged.get(target_index, 0.0) + weight

        # The vanilla head stores the highest weight first.  The name
        # tie-breaker preserves deterministic ordering for equal weights.
        ordered = sorted(
            merged.items(),
            key=lambda item: (item[1], destination_names[item[0]]),
            reverse=True,
        )
        indices = [index for index, _weight in ordered]
        weights = [weight for _index, weight in ordered]
        weights.extend([0.0] * (4 - len(weights)))
        indices.extend([0] * (4 - len(indices)))
        remap_data.extend(REMAP_RECORD.pack(*weights, *indices))

    new_bones_data = bytearray()
    for name, source_index in appended:
        encoded = name.encode("utf-8")
        if len(encoded) >= NEW_BONE_NAME_SIZE:
            raise FaceBonesError(
                f"bone name '{name}' is too long for the 0x80-byte field"
            )
        new_bones_data.extend(encoded)
        new_bones_data.extend(b"\0" * (NEW_BONE_NAME_SIZE - len(encoded)))
        new_bones_data.extend(
            _disk_bone_transform_to_memory(main.bone_data[source_index])
        )

    return GenerationResult(
        main_shape_name=main_shape.name,
        face_shape_name=face_shape.name,
        vertex_count=main_shape.vertex_count,
        main_bone_count=len(main.palette),
        face_bone_count=len(face.palette),
        appended_bones=tuple(name for name, _source_index in appended),
        collapsed_bones=tuple(collapsed),
        remap_data=bytes(remap_data),
        new_bones_data=bytes(new_bones_data),
    )


def _safe_geometry_filename(geometry_name: str) -> str:
    name = re.sub(r'[<>:"/\\|?*]', "_", geometry_name).strip(" .")
    if not name:
        raise FaceBonesError("the FaceBones geometry has no usable filename")
    return name


def output_paths(geometry_name: str, output_directory: os.PathLike[str] | str) -> tuple[Path, Path]:
    output = Path(output_directory)
    stem = _safe_geometry_filename(geometry_name)
    return (
        output / f"{stem}_CustomizationRemapData.bin",
        output / f"{stem}_CustomizationRemapNewBonesData.bin",
    )


def _atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(handle, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            temporary.unlink(missing_ok=True)
        finally:
            raise


def write_payloads(result: GenerationResult, output_directory: os.PathLike[str] | str) -> tuple[Path, Path]:
    remap_path, new_bones_path = output_paths(
        result.face_shape_name, output_directory
    )
    _atomic_write(remap_path, result.remap_data)
    _atomic_write(new_bones_path, result.new_bones_data)
    return remap_path, new_bones_path


def run_gui() -> None:
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk

    root = tk.Tk()
    root.title("Fallout 4 FaceBones Extra Data Generator")
    root.minsize(760, 245)

    main_value = tk.StringVar()
    face_value = tk.StringVar()
    output_value = tk.StringVar()
    status_value = tk.StringVar(value="Select the finalized exported NIF files.")

    frame = ttk.Frame(root, padding=14)
    frame.grid(row=0, column=0, sticky="nsew")
    root.columnconfigure(0, weight=1)
    root.rowconfigure(0, weight=1)
    frame.columnconfigure(1, weight=1)

    def initial_directory(value: str) -> str | None:
        path = Path(value.strip().strip('"')) if value.strip() else None
        if path is None:
            return None
        if path.is_dir():
            return str(path)
        if path.parent.is_dir():
            return str(path.parent)
        return None

    def browse_nif(target: tk.StringVar) -> None:
        selected = filedialog.askopenfilename(
            parent=root,
            title="Select Fallout 4 NIF",
            initialdir=initial_directory(target.get()),
            filetypes=(("NIF files", "*.nif"), ("All files", "*.*")),
        )
        if selected:
            target.set(selected)
            if target is face_value and not output_value.get().strip():
                output_value.set(str(Path(selected).parent))

    def browse_output() -> None:
        selected = filedialog.askdirectory(
            parent=root,
            title="Select output directory",
            initialdir=initial_directory(output_value.get()),
            mustexist=False,
        )
        if selected:
            output_value.set(selected)

    rows = (
        ("Main NIF", main_value, lambda: browse_nif(main_value)),
        ("FaceBones NIF", face_value, lambda: browse_nif(face_value)),
        ("Output directory", output_value, browse_output),
    )
    for row, (label, variable, command) in enumerate(rows):
        ttk.Label(frame, text=label).grid(row=row, column=0, padx=(0, 10), pady=6, sticky="w")
        ttk.Entry(frame, textvariable=variable).grid(row=row, column=1, pady=6, sticky="ew")
        ttk.Button(frame, text="Browse...", command=command).grid(
            row=row, column=2, padx=(10, 0), pady=6
        )

    ttk.Separator(frame).grid(row=3, column=0, columnspan=3, pady=(12, 10), sticky="ew")
    ttk.Label(frame, textvariable=status_value, wraplength=710).grid(
        row=4, column=0, columnspan=3, sticky="w"
    )

    def generate() -> None:
        main_path = Path(main_value.get().strip().strip('"'))
        face_path = Path(face_value.get().strip().strip('"'))
        output_directory = Path(output_value.get().strip().strip('"'))
        if not main_value.get().strip() or not face_value.get().strip() or not output_value.get().strip():
            messagebox.showerror(
                "Missing input",
                "Select the main NIF, FaceBones NIF, and output directory.",
                parent=root,
            )
            return

        try:
            result = generate_payloads(main_path, face_path)
            remap_path, new_bones_path = output_paths(
                result.face_shape_name, output_directory
            )
            existing = [path for path in (remap_path, new_bones_path) if path.exists()]
            if existing and not messagebox.askyesno(
                "Replace existing files?",
                "The following output files already exist:\n\n"
                + "\n".join(str(path) for path in existing)
                + "\n\nReplace them?",
                parent=root,
            ):
                status_value.set("Generation cancelled; no files were changed.")
                return
            remap_path, new_bones_path = write_payloads(result, output_directory)
        except (FaceBonesError, OSError, struct.error, UnicodeError) as exc:
            status_value.set(f"Error: {exc}")
            messagebox.showerror("Generation failed", str(exc), parent=root)
            return

        appended = ", ".join(result.appended_bones) or "none"
        collapsed = ", ".join(
            f"{bone} -> {ancestor}"
            for bone, ancestor in result.collapsed_bones
        ) or "none"
        status_value.set(
            f"Generated {result.vertex_count} remap records and "
            f"{len(result.appended_bones)} new-bone records; "
            f"collapsed {len(result.collapsed_bones)} unavailable bones."
        )
        messagebox.showinfo(
            "Generation complete",
            f"Main geometry: {result.main_shape_name}\n"
            f"FaceBones geometry: {result.face_shape_name}\n"
            f"Vertices: {result.vertex_count}\n"
            f"Appended bones: {appended}\n\n"
            f"Collapsed bones: {collapsed}\n\n"
            f"Wrote:\n{remap_path}\n{new_bones_path}",
            parent=root,
        )

    generate_button = ttk.Button(frame, text="Generate", command=generate)
    generate_button.grid(row=5, column=0, columnspan=3, pady=(16, 0), ipadx=30, ipady=4)
    root.bind("<Return>", lambda _event: generate_button.invoke())

    root.mainloop()


if __name__ == "__main__":
    run_gui()
