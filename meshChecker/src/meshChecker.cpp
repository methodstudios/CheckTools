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

#include <limits>
#include <cmath>

using IndexArray = MeshChecker::IndexArray;

namespace
{

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
    UNDEFINED // keep last
};

enum class ResultType
{
    Face,
    Vertex,
    Edge,
    UV
};

MStringArray create_result_string(const MDagPath& path, const IndexArray& indices, ResultType type)
{
    MString full_path = path.fullPathName();
    std::vector<MString> result;
    result.reserve(indices.size());

    for(auto index : indices)
    {
        switch(type)
        {
            case ResultType::Face:
            {
                result.emplace_back(full_path + ".f[" + index + "]");
                break;
            }
            case ResultType::Vertex:
            {
                result.emplace_back(full_path + ".vtx[" + index + "]");
                break;
            }

            case ResultType::Edge:
            {
                result.emplace_back(full_path + ".e[" + index + "]");
                break;
            }
            case ResultType::UV:
            {
                result.emplace_back(full_path + ".map[" + index + "]");
                break;
            }
        }
    }
    return {&result[0], static_cast<unsigned int>(indices.size())};
}

} // namespace


MeshChecker::MeshChecker()
    : MPxCommand()
{
}

IndexArray MeshChecker::findTriangles(const MFnMesh& mesh)
{
    auto num_polygons = mesh.numPolygons();

    IndexArray indices;
    indices.reserve(static_cast<size_t>(num_polygons));

    for(Index poly_index{}; poly_index<num_polygons; ++poly_index)
    {
        if(mesh.polygonVertexCount(poly_index) == 3)
        {
            indices.push_back(poly_index);
        }
    }
    return indices;
}

IndexArray MeshChecker::findNgons(const MFnMesh& mesh)
{
    auto num_polygons = mesh.numPolygons();

    IndexArray indices;
    indices.reserve(static_cast<size_t>(num_polygons));

    for(Index poly_index{}; poly_index<num_polygons; ++poly_index)
    {
        if(mesh.polygonVertexCount(poly_index) >= 5)
        {
            indices.push_back(poly_index);
        }
    }
    return indices;
}

IndexArray MeshChecker::findNonManifoldEdges(const MFnMesh& mesh)
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
            indices.push_back(edge_it.index());
        }
    }
    return indices;
}

IndexArray MeshChecker::findLaminaFaces(const MFnMesh& mesh)
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

IndexArray MeshChecker::findBiValentFaces(const MFnMesh& mesh)
{
    MDagPath path;
    mesh.getPath(path);

    IndexArray indices;
    indices.reserve(static_cast<size_t>(mesh.numVertices()));

    MIntArray connectedFaces;
    MIntArray connectedEdges;

    for (MItMeshVertex vertex_it(path); !vertex_it.isDone(); vertex_it.next())
    {
        vertex_it.getConnectedFaces(connectedFaces);
        vertex_it.getConnectedEdges(connectedEdges);

        if (connectedFaces.length() == 2 && connectedEdges.length() == 2)
        {
            indices.push_back(vertex_it.index());
        }
    }
    return indices;
}


IndexArray MeshChecker::findZeroAreaFaces(const MFnMesh& mesh, double maxFaceArea)
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


IndexArray MeshChecker::findMeshBorderEdges(const MFnMesh& mesh)
{
    MDagPath path;
    mesh.getPath(path);

    IndexArray indices;
    indices.reserve(static_cast<size_t>(mesh.numEdges()));

    for (MItMeshEdge edge_it(path); !edge_it.isDone(); edge_it.next())
    {
        if (edge_it.onBoundary())
        {
            indices.push_back(edge_it.index());
        }
    }
    return indices;
}

IndexArray MeshChecker::findCreaseEdges(const MFnMesh& mesh)
{
    MUintArray edgeIds;
    MDoubleArray creaseData;
    mesh.getCreaseEdges(edgeIds, creaseData);

    IndexArray indices;
    indices.reserve(static_cast<size_t>(edgeIds.length()));

    for (unsigned int i{}; i < edgeIds.length(); i++)
    {
        if (creaseData[i] != 0)
        {
            indices.push_back(static_cast<Index>(edgeIds[i]));
        }
    }
    return indices;
}

IndexArray MeshChecker::findZeroLengthEdges(const MFnMesh& mesh, double minEdgeLength)
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
            indices.push_back(edge_it.index());
        }
    }
    return indices;
}


IndexArray MeshChecker::findUnfrozenVertices(const MFnMesh& mesh)
{
    MDagPath path;
    mesh.getPath(path);

    path.extendToShape();
    MFnDagNode dag_node{path};
    MPlug pnts_plug = dag_node.findPlug("pnts");

    auto num_vertices = mesh.numVertices();
    IndexArray indices;
    indices.reserve(static_cast<size_t>(num_vertices));

    for(Index i{}; i<num_vertices; ++i)
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
            if(!(std::abs(xyz[0]) <= eps &&
                 std::abs(xyz[1]) <= eps &&
                 std::abs(xyz[2]) <= eps))
            {
                indices.push_back(i);
            }
        }
    }

    return indices;
}


bool MeshChecker::hasVertexPntsAttr(const MFnMesh& mesh, bool fix)
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

        while (true) {
            outputHandle = arrayDataHandle.outputValue();


            float3& xyz = outputHandle.asFloat3();
            if (xyz) {
                if (xyz[0] != 0.0)
                    pntsArray.destructHandle(dataHandle);
                return true;
                if (xyz[1] != 0.0)
                    pntsArray.destructHandle(dataHandle);
                return true;
                if (xyz[2] != 0.0)
                    pntsArray.destructHandle(dataHandle);
                return true;
            }
            status = arrayDataHandle.next();
            if (status != MS::kSuccess) {
                break;
            }
        }
    }
    else {
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
            auto indices = findTriangles(mesh);
            setResult(create_result_string(path, indices, ResultType::Face));
        }
        else if(check_type == MeshCheckType::NGONS)
        {
            auto indices = findNgons(mesh);
            setResult(create_result_string(path, indices, ResultType::Face));
        }
        else if(check_type == MeshCheckType::NON_MANIFOLD_EDGES)
        {
            auto indices = findNonManifoldEdges(mesh);
            setResult(create_result_string(path, indices, ResultType::Edge));
        }
        else if(check_type == MeshCheckType::LAMINA_FACES)
        {
            auto indices = findLaminaFaces(mesh);
            setResult(create_result_string(path, indices, ResultType::Face));
        }
        else if(check_type == MeshCheckType::BI_VALENT_FACES)
        {
            auto indices = findBiValentFaces(mesh);
            setResult(create_result_string(path, indices, ResultType::Vertex));
        }
        else if(check_type == MeshCheckType::ZERO_AREA_FACES)
        {
            if (args_database.isFlagSet("-maxFaceArea")) args_database.getFlagArgument("-maxFaceArea", 0, max_tolerance);

            auto indices = findZeroAreaFaces(mesh, max_tolerance);
            setResult(create_result_string(path, indices, ResultType::Face));
        }
        else if(check_type == MeshCheckType::MESH_BORDER)
        {
            auto indices = findMeshBorderEdges(mesh);
            setResult(create_result_string(path, indices, ResultType::Edge));
        }
        else if(check_type == MeshCheckType::CREASE_EDGE)
        {
            auto indices = findCreaseEdges(mesh);
            setResult(create_result_string(path, indices, ResultType::Edge));
        }
        else if(check_type == MeshCheckType::ZERO_LENGTH_EDGES)
        {
            if (args_database.isFlagSet("-minEdgeLength")) args_database.getFlagArgument("-minEdgeLength", 0, max_tolerance);

            auto indices = findZeroLengthEdges(mesh, max_tolerance);
            setResult(create_result_string(path, indices, ResultType::Edge));
        }
        else if(check_type == MeshCheckType::UNFROZEN_VERTICES)
        {
            bool fix = false;
            if (args_database.isFlagSet("-fix")) args_database.getFlagArgument("-fix", 0, fix);

            setResult(hasVertexPntsAttr(mesh, fix));
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

void* MeshChecker::creator()
{
    return new MeshChecker;
}

MSyntax MeshChecker::newSyntax()
{
    MSyntax syntax;
    syntax.setObjectType(MSyntax::kSelectionList);
    syntax.addFlag("-c", "-check", MSyntax::kUnsigned);
    syntax.addFlag("-mfa", "-maxFaceArea", MSyntax::kDouble);
    syntax.addFlag("-mel", "-minEdgeLength", MSyntax::kDouble);
    syntax.addFlag("-fix", "-doFix", MSyntax::kBoolean);

//    syntax.addFlag("-f", "-find", MSyntax::kString);
//    syntax.addFlag("-t", "-tolerance", MSyntax::kDouble);
    return syntax;
}
