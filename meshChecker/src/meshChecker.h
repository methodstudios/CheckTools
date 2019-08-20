#ifndef MESHCHECKER_H
#define MESHCHECKER_H

#include <maya/MApiNamespace.h>
#include <maya/MPxCommand.h>

#include <vector>

using Index = size_t;
using IndexArray = std::vector<Index>;

class MeshChecker final : public MPxCommand
{
public:
    // Factories
    static void* Creator();
    static MSyntax NewSyntax();

    // MPxCommand interface
    MStatus doIt(const MArgList& argList) final;
    MStatus undoIt() final;
    MStatus redoIt() final;
    bool isUndoable() const final;

    // Operations
    static IndexArray FindTriangles(const MFnMesh &mesh);
    static IndexArray FindNGons(const MFnMesh &mesh);
    static IndexArray FindNonManifoldEdges(const MFnMesh &mesh);
    static IndexArray FindLaminaFaces(const MFnMesh &mesh);
    static IndexArray FindBiValentFaces(const MFnMesh &mesh);
    static IndexArray FindZeroAreaFaces(const MFnMesh &mesh, double maxFaceArea);
    static IndexArray FindMeshBorderEdges(const MFnMesh &mesh);
    static IndexArray FindCreaseEdges(const MFnMesh &mesh);
    static IndexArray FindZeroLengthEdges(const MFnMesh &mesh, double minEdgeLength);
    static IndexArray FindUnfrozenVertices(const MFnMesh &mesh);
    static IndexArray FindOverlappingFaces(const MFnMesh &mesh);
    static bool HasVertexPntsAttr(const MFnMesh &, bool fix);

private:
    MeshChecker();
};

#endif // MESHCHECKER_H
