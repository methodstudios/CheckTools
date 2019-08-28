#include "meshChecker.h"

#include <maya/MArgDatabase.h>
#include <maya/MArgList.h>
#include <maya/MDoubleArray.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnMesh.h>
#include <maya/MGlobal.h>
#include <maya/MPlug.h>
#include <maya/MString.h>
#include <maya/MSyntax.h>
#include <maya/MUintArray.h>
#include <maya/MDataHandle.h>
#include <maya/MSelectionList.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MItMeshEdge.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MItMeshVertex.h>
#include <maya/MDagPath.h>
#include <maya/MFloatPointArray.h>
#include <maya/MBoundingBox.h>
#include <maya/MPointArray.h>
#include <maya/MMeshIntersector.h>

#include <tbb/tbb.h>

#include <limits>
#include <cmath>
#include <array>
#include <set>
#include <algorithm>
#include <string>

namespace
{

Index operator"" _i(unsigned long long int v)
{
    return static_cast<Index>(v);
}

MString operator"" _ms(const char* v, size_t size)
{
    return {v, static_cast<int>(size)};
}

enum class MeshCheckType
{
    TRIANGLES = 0,
    NGONS,
    NON_MANIFOLD_EDGES,
    LAMINA_FACES,
    BI_VALENT_FACES,
    ZERO_AREA_FACES,
    MESH_BORDER,
    CREASE_EDGE,
    ZERO_LENGTH_EDGES,
    UNFROZEN_VERTICES,
    OVERLAPPING_FACES,
    UNDEFINED // keep last
};

enum class ComponentType
{
    Face,
    Vertex,
    Edge,
    UV
};

MStringArray create_result_string(const MDagPath& path, const IndexArray& indices, ComponentType type)
{
    const MString full_path = path.fullPathName();
    std::vector<MString> result;
    result.reserve(indices.size());

    for(auto index : indices)
    {
        auto index_str = std::to_string(index);

        switch(type)
        {
            case ComponentType::Face:
            {
                result.emplace_back(full_path + ".f[" + index_str.c_str() + "]");
                break;
            }
            case ComponentType::Vertex:
            {
                result.emplace_back(full_path + ".vtx[" + index_str.c_str() + "]");
                break;
            }
            case ComponentType::Edge:
            {
                result.emplace_back(full_path + ".e[" + index_str.c_str() + "]");
                break;
            }
            case ComponentType::UV:
            {
                result.emplace_back(full_path + ".map[" + index_str.c_str() + "]");
                break;
            }
        }
    }

    //
    return {&result[0], static_cast<unsigned int>(indices.size())};
}

} // namespace

MeshChecker::MeshChecker()
    : MPxCommand()
{
}

IndexArray MeshChecker::FindTriangles(const MFnMesh& mesh)
{
    auto num_polygons = static_cast<Index>(mesh.numPolygons());

    IndexArray indices;
    indices.reserve(num_polygons);

    for(auto poly_index = 0_i; poly_index<num_polygons; ++poly_index)
    {

        if(mesh.polygonVertexCount(static_cast<int>(poly_index)) == 3)
        {
            indices.push_back(poly_index);
        }
    }
    return indices;
}

IndexArray MeshChecker::FindNGons(const MFnMesh& mesh)
{
    auto num_polygons = static_cast<Index>(mesh.numPolygons());

    IndexArray indices;
    indices.reserve(static_cast<size_t>(num_polygons));

    for(auto poly_index = 0_i; poly_index<num_polygons; ++poly_index)
    {
        if(mesh.polygonVertexCount(static_cast<int>(poly_index)) >= 5)
        {
            indices.push_back(poly_index);
        }
    }
    return indices;
}

IndexArray MeshChecker::FindNonManifoldEdges(const MFnMesh& mesh)
{
    MDagPath path;
    mesh.getPath(path);

    IndexArray indices;
    indices.reserve(static_cast<size_t>(mesh.numEdges()));

    for (MItMeshEdge edge_it(path); !edge_it.isDone(); edge_it.next())
    {
        int face_count;
        edge_it.numConnectedFaces(face_count);
        if (face_count > 2)
        {
            indices.push_back(static_cast<Index>(edge_it.index()));
        }
    }
    return indices;
}

IndexArray MeshChecker::FindLaminaFaces(const MFnMesh& mesh)
{
    MDagPath path;
    mesh.getPath(path);

    IndexArray indices;
    indices.reserve(static_cast<size_t>(mesh.numPolygons()));

    for (MItMeshPolygon poly_it(path); !poly_it.isDone(); poly_it.next())
    {
        if (poly_it.isLamina())
        {
            indices.push_back(static_cast<Index>(poly_it.index()));
        }
    }
    return indices;
}

IndexArray MeshChecker::FindBiValentFaces(const MFnMesh& mesh)
{
    MDagPath path;
    mesh.getPath(path);

    IndexArray indices;
    indices.reserve(static_cast<size_t>(mesh.numVertices()));

    MIntArray connectedFaces, connectedEdges;
    for (MItMeshVertex vertex_it(path); !vertex_it.isDone(); vertex_it.next())
    {
        vertex_it.getConnectedFaces(connectedFaces);
        vertex_it.getConnectedEdges(connectedEdges);

        if (connectedFaces.length() == 2 && connectedEdges.length() == 2)
        {
            indices.push_back(static_cast<Index>(vertex_it.index()));
        }
    }
    return indices;
}

IndexArray MeshChecker::FindZeroAreaFaces(const MFnMesh& mesh, double maxFaceArea)
{
    MDagPath path;
    mesh.getPath(path);

    IndexArray indices;
    indices.reserve(static_cast<size_t>(mesh.numPolygons()));

    for (MItMeshPolygon poly_it(path); !poly_it.isDone(); poly_it.next())
    {
        double area;
        poly_it.getArea(area);
        if (area < maxFaceArea)
        {
            indices.push_back(static_cast<Index>(poly_it.index()));
        }
    }
    return indices;
}

IndexArray MeshChecker::FindMeshBorderEdges(const MFnMesh& mesh)
{
    MDagPath path;
    mesh.getPath(path);

    IndexArray indices;
    indices.reserve(static_cast<size_t>(mesh.numEdges()));

    for (MItMeshEdge edge_it(path); !edge_it.isDone(); edge_it.next())
    {
        if (edge_it.onBoundary())
        {
            indices.push_back(static_cast<Index>(edge_it.index()));
        }
    }
    return indices;
}

IndexArray MeshChecker::FindCreaseEdges(const MFnMesh& mesh)
{
    MUintArray edgeIds;
    MDoubleArray creaseData;
    mesh.getCreaseEdges(edgeIds, creaseData);

    IndexArray indices;
    indices.reserve(static_cast<size_t>(edgeIds.length()));

    for (auto i = 0_i; i < edgeIds.length(); i++)
    {
        auto index = static_cast<unsigned int>(i);

        if (creaseData[index] != 0)
        {
            indices.push_back(static_cast<Index>(edgeIds[index]));
        }
    }
    return indices;
}

IndexArray MeshChecker::FindZeroLengthEdges(const MFnMesh& mesh, double minEdgeLength)
{
    MDagPath path;
    mesh.getPath(path);

    IndexArray indices;
    indices.reserve(static_cast<size_t>(mesh.numEdges()));

    for (MItMeshEdge edge_it(path); !edge_it.isDone(); edge_it.next())
    {
        double length;
        edge_it.getLength(length);
        if (length < minEdgeLength)
        {
            indices.push_back(static_cast<Index>(edge_it.index()));
        }
    }
    return indices;
}

IndexArray MeshChecker::FindUnfrozenVertices(const MFnMesh& mesh)
{
    MDagPath path;
    mesh.getPath(path);

    path.extendToShape();
    MFnDagNode dag_node{path};
    MPlug pnts_plug = dag_node.findPlug("pnts");

    auto num_vertices = static_cast<Index>(mesh.numVertices());
    IndexArray indices;
    indices.reserve(num_vertices);

    for(auto i = 0_i; i<num_vertices; ++i)
    {
        MPlug xyz_plug = pnts_plug.elementByLogicalIndex(static_cast<unsigned int>(i));
        if (xyz_plug.isCompound())
        {
            float xyz[3];
            for(unsigned int j{}; j<3; ++j)
            {
                xyz_plug.child(j).getValue(xyz[j]);
            }

            auto eps = std::numeric_limits<float>::epsilon();
            if(!(std::abs(xyz[0]) <= eps && std::abs(xyz[1]) <= eps && std::abs(xyz[2]) <= eps))
            {
                indices.push_back(i);
            }
        }
    }

    return indices;
}

IndexArray MeshChecker::FindOverlappingFaces(const MFnMesh& mesh)
{
    // result
    IndexArray index_array;
    index_array.reserve(static_cast<size_t>(mesh.numVertices()));

    MFloatPointArray points;
    mesh.getPoints(points);

    MIntArray vertex_count, vertex_list;
    mesh.getVertices(vertex_count, vertex_list);

    std::vector<size_t> vertex_offset{};
    vertex_offset.reserve(vertex_count.length());
    for (size_t i{}, offset{}; i < vertex_count.length(); offset+=vertex_count[i], ++i)
    {
        vertex_offset.push_back(offset);
    }

    //
    // Compute rounded positions
    //
    using Vector4 = std::array<long long, 3>;
    using Vector4Array = std::vector<Vector4>;

    Vector4Array rounded_pos;
    rounded_pos.resize(vertex_list.length());

    tbb::parallel_for(tbb::blocked_range<size_t>(0, static_cast<size_t>(mesh.numPolygons())),
          [&] (const tbb::blocked_range<size_t>& r)
          {
              for(auto poly_id = r.begin(); poly_id<r.end(); ++poly_id)
              {
                  const size_t poly_vertex_count = vertex_count[poly_id];
                  const size_t poly_vertex_offset = vertex_offset[poly_id];

                  for(size_t i = poly_vertex_offset; i < poly_vertex_offset + poly_vertex_count; ++i)
                  {
                      const size_t point_offset = vertex_list[i];
                      const MFloatPoint& pos = points[point_offset];

                      auto x = static_cast<const Vector4::value_type>(pos.x * 100000);
                      auto y = static_cast<const Vector4::value_type>(pos.y * 100000);
                      auto z = static_cast<const Vector4::value_type>(pos.z * 100000);
                      rounded_pos[i] = Vector4{x,y,z};
                  }

                  // sort positions
                  std::sort(rounded_pos.begin() + poly_vertex_offset,
                            rounded_pos.begin() + poly_vertex_offset + poly_vertex_count);
              }
          });

    std::set<Vector4Array> face_set;
    for (size_t poly_id{}, prev_length{}; poly_id < mesh.numPolygons(); ++poly_id)
    {
        const size_t poly_vertex_count = vertex_count[poly_id];
        const size_t poly_vertex_offset = vertex_offset[poly_id];

        auto local = Vector4Array{rounded_pos.begin() + poly_vertex_offset,
                                  rounded_pos.begin() + poly_vertex_offset + poly_vertex_count};

        face_set.insert(std::move(local));

        if (prev_length == face_set.size())
        {
            index_array.push_back(static_cast<Index>(poly_id));
        }
        prev_length = static_cast<int>(face_set.size());
    }

    return index_array;
}

bool MeshChecker::HasVertexPntsAttr(const MFnMesh& mesh, bool fix)
{
    MDagPath path;
    mesh.getPath(path);

    MStatus status;

    path.extendToShape();
    MFnDagNode dagNode(path);
    MPlug pntsArray = dagNode.findPlug("pnts");
    MDataHandle dataHandle = pntsArray.asMDataHandle();
    MArrayDataHandle arrayDataHandle(dataHandle);
    MDataHandle outputHandle;

    if (!fix) {
        // Check only.

        while (true)
        {
            outputHandle = arrayDataHandle.outputValue();

            float3& xyz = outputHandle.asFloat3();
            if (xyz) {
                if (xyz[0] != 0.0) {
                    pntsArray.destructHandle(dataHandle);
                    return true;
                }
                if (xyz[1] != 0.0) {
                    pntsArray.destructHandle(dataHandle);
                    return true;
                }
                if (xyz[2] != 0.0) {
                    pntsArray.destructHandle(dataHandle);
                    return true;
                }
            }
            status = arrayDataHandle.next();
            if (status != MS::kSuccess) {
                break;
            }
        }
    }
    else
    {
        // Do fix. Reset all vertices pnts attr to 0
        MObject pntx = dagNode.attribute("pntx");
        MObject pnty = dagNode.attribute("pnty");
        MObject pntz = dagNode.attribute("pntz");
        MDataHandle xHandle;
        MDataHandle yHandle;
        MDataHandle zHandle;

        while (true) {
            outputHandle = arrayDataHandle.outputValue();

            // outputHandle.set3Double(0.0, 0.0, 0.0);

            // setting 3 values at the same time kills maya in
            // some environments somehow. So here setting values separately
            xHandle = outputHandle.child(pntx);
            yHandle = outputHandle.child(pnty);
            zHandle = outputHandle.child(pntz);
            xHandle.setFloat(0.0);
            yHandle.setFloat(0.0);
            zHandle.setFloat(0.0);

            status = arrayDataHandle.next();
            if (status != MS::kSuccess) {
                break;
            }
        }
        pntsArray.setMDataHandle(dataHandle);
    }
    pntsArray.destructHandle(dataHandle);
    return false;
}

MStatus MeshChecker::doIt(const MArgList &args)
{
    MStatus status;
    MArgDatabase args_database(syntax(), args);

    // -check arg
    MeshCheckType check_type;
    if (!args_database.isFlagSet("-check"))
    {
        MGlobal::displayError("-check argument is required");
        return MS::kFailure;
    }
    else
    {
        // process check
        unsigned int check_value;
        status = args_database.getFlagArgument("-check", 0, check_value);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        if(check_value >= static_cast<unsigned int>(MeshCheckType::UNDEFINED))
        {
            MGlobal::displayError("Invalid check.");
            return MS::kFailure;
        }
        check_type = static_cast<MeshCheckType>(check_value);
    }

    // if argument is not provided use selection list
    MSelectionList selection;
    args_database.getObjects(selection);

    if(!selection.length())
    {
        status = MGlobal::getActiveSelectionList(selection);
        CHECK_MSTATUS_AND_RETURN_IT(status);
    }

    if(selection.length() == 0)
    {
        MGlobal::displayError("Invalid selection.");
        return MStatus::kFailure;
    }

    for(unsigned int i{}; i<selection.length(); ++i)
    {
        MDagPath path;
        selection.getDagPath(i, path);

        MFnMesh mesh{path, &status};
        if(!status)
        {
            MGlobal::displayWarning("MeshCheker works on meshes.");
            continue;
        }

        double max_tolerance{0.000001};

        // execute operation
        if(check_type == MeshCheckType::TRIANGLES)
        {
            auto indices = FindTriangles(mesh);
            setResult(create_result_string(path, indices, ComponentType::Face));
        }
        else if(check_type == MeshCheckType::NGONS)
        {
            auto indices = FindNGons(mesh);
            setResult(create_result_string(path, indices, ComponentType::Face));
        }
        else if(check_type == MeshCheckType::NON_MANIFOLD_EDGES)
        {
            auto indices = FindNonManifoldEdges(mesh);
            setResult(create_result_string(path, indices, ComponentType::Edge));
        }
        else if(check_type == MeshCheckType::LAMINA_FACES)
        {
            auto indices = FindLaminaFaces(mesh);
            setResult(create_result_string(path, indices, ComponentType::Face));
        }
        else if(check_type == MeshCheckType::BI_VALENT_FACES)
        {
            auto indices = FindBiValentFaces(mesh);
            setResult(create_result_string(path, indices, ComponentType::Vertex));
        }
        else if(check_type == MeshCheckType::ZERO_AREA_FACES)
        {
            if (args_database.isFlagSet("-maxFaceArea")) args_database.getFlagArgument("-maxFaceArea", 0, max_tolerance);

            auto indices = FindZeroAreaFaces(mesh, max_tolerance);
            setResult(create_result_string(path, indices, ComponentType::Face));
        }
        else if(check_type == MeshCheckType::MESH_BORDER)
        {
            auto indices = FindMeshBorderEdges(mesh);
            setResult(create_result_string(path, indices, ComponentType::Edge));
        }
        else if(check_type == MeshCheckType::CREASE_EDGE)
        {
            auto indices = FindCreaseEdges(mesh);
            setResult(create_result_string(path, indices, ComponentType::Edge));
        }
        else if(check_type == MeshCheckType::ZERO_LENGTH_EDGES)
        {
            if (args_database.isFlagSet("-minEdgeLength")) args_database.getFlagArgument("-minEdgeLength", 0, max_tolerance);

            auto indices = FindZeroLengthEdges(mesh, max_tolerance);
            setResult(create_result_string(path, indices, ComponentType::Edge));
        }
        else if(check_type == MeshCheckType::UNFROZEN_VERTICES)
        {
            bool fix = false;
            if (args_database.isFlagSet("-fix")) args_database.getFlagArgument("-fix", 0, fix);

            setResult(HasVertexPntsAttr(mesh, fix));
        }
        else if(check_type == MeshCheckType::OVERLAPPING_FACES)
        {
            auto indices = FindOverlappingFaces(mesh); // self intersection
            setResult(create_result_string(path, indices, ComponentType::Face));
        }
        else
        {
            MGlobal::displayWarning("Invalid check number!");
            continue;
        }
    }

    return redoIt();
}

MStatus MeshChecker::redoIt()
{
    return MS::kSuccess;
}

MStatus MeshChecker::undoIt()
{
    return MS::kSuccess;
}

bool MeshChecker::isUndoable() const
{
    return false;
}

void* MeshChecker::Creator()
{
    return new MeshChecker{};
}

MSyntax MeshChecker::NewSyntax()
{
    MSyntax syntax;
    syntax.setObjectType(MSyntax::kSelectionList);
    syntax.addFlag("-c", "-check", MSyntax::kUnsigned);
    syntax.addFlag("-mfa", "-maxFaceArea", MSyntax::kDouble);
    syntax.addFlag("-mel", "-minEdgeLength", MSyntax::kDouble);
    syntax.addFlag("-fix", "-doFix", MSyntax::kBoolean);

    //syntax.addFlag("-f", "-find", MSyntax::kString);
//    syntax.addFlag("-t", "-tolerance", MSyntax::kDouble);
    return syntax;
}
