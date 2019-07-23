#ifndef __MESHCHECKER_H__
#define __MESHCHECKER_H__

#include <maya/MApiNamespace.h>
#include <maya/MPxCommand.h>

#include <vector>


class MeshChecker final : public MPxCommand
{
public:
    static void* creator();
    static MSyntax newSyntax();

    // command interface
    MStatus doIt(const MArgList& argList) final;
    MStatus undoIt() final;
    MStatus redoIt() final;
    bool isUndoable() const final;

    // aliases
    using Index = int;
    using IndexArray = std::vector<Index>;

    // operations
    static IndexArray findTriangles(const MFnMesh&);
    static IndexArray findNgons(const MFnMesh&);
    static IndexArray findNonManifoldEdges(const MFnMesh&);
    static IndexArray findLaminaFaces(const MFnMesh&);
    static IndexArray findBiValentFaces(const MFnMesh&);
    static IndexArray findZeroAreaFaces(const MFnMesh&, double maxFaceArea);
    static IndexArray findMeshBorderEdges(const MFnMesh&);
    static IndexArray findCreaseEdges(const MFnMesh&);
    static IndexArray findZeroLengthEdges(const MFnMesh&, double minEdgeLength);
    static IndexArray findUnfrozenVertices(const MFnMesh&);
    static IndexArray findOverlappingFaces(const MFnMesh&);
    static bool hasVertexPntsAttr(const MFnMesh&, bool fix);

private:
    MeshChecker();
};

#endif /* defined(__MESHCHECKER_H__) */
