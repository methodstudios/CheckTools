#include "uvChecker.h"
#include <cstddef>
#include <math.h>
#include <maya/MArgDatabase.h>
#include <maya/MArgList.h>
#include <maya/MFnMesh.h>
#include <maya/MGlobal.h>
#include <maya/MString.h>
#include <maya/MIntArray.h>
#include <maya/MFloatArray.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MSelectionList.h>
#include <maya/MStringArray.h>
#include <set>
#include <vector>
#include <algorithm>
#include <string>

UvChecker::UvChecker()
{
}

UvChecker::~UvChecker()
{
}

MSyntax UvChecker::newSyntax()
{
    MSyntax syntax;
    syntax.addArg(MSyntax::kString);
    syntax.addFlag("-v", "-verbose", MSyntax::kBoolean);
    syntax.addFlag("-c", "-check", MSyntax::kUnsigned);
    syntax.addFlag("-uva", "-uvArea", MSyntax::kDouble);
    return syntax;
}

MStatus UvChecker::doIt(const MArgList& args)
{
    MStatus status;

    MSelectionList sel;

    MArgDatabase argData(syntax(), args);

    status = argData.getCommandArgument(0, sel);
    if (status != MS::kSuccess) {
        MGlobal::displayError("You have to provide an object path");
        return MStatus::kFailure;
    }

    if (argData.isFlagSet("-verbose"))
        argData.getFlagArgument("-verbose", 0, verbose);
    else
        verbose = false;

    if (argData.isFlagSet("-check"))
        argData.getFlagArgument("-check", 0, checkNumber);
    else
        checkNumber = 99;

    if (argData.isFlagSet("-uvArea"))
        argData.getFlagArgument("-uvArea", 0, minUVArea);
    else
        minUVArea = 0.000001;

    sel.getDagPath(0, mDagPath);

    if (verbose == true) {
        MString objectPath = "Selected mesh : " + mDagPath.fullPathName();
        MGlobal::displayInfo(objectPath);
    }

    status = mDagPath.extendToShape();
    CHECK_MSTATUS_AND_RETURN_IT(status);

    if (mDagPath.apiType() != MFn::kMesh) {
        MGlobal::displayError("Selected object is not mesh.");
        return MStatus::kFailure;
    }

    return redoIt();
}

MStatus UvChecker::redoIt()
{
    MStatus status;

    switch (checkNumber) {
    case UvChecker::UDIM:
        if (verbose == true) {
            MGlobal::displayInfo("Checking UDIM borders");
        }
        status = findUdimIntersections();
        CHECK_MSTATUS_AND_RETURN_IT(status);
        break;
    case UvChecker::HAS_UVS:
        if (verbose == true) {
            MGlobal::displayInfo("Checking Non UVed faces");
        }
        status = findNoUvFaces();
        CHECK_MSTATUS_AND_RETURN_IT(status);
        break;
    case UvChecker::ZERO_AREA:
        if (verbose == true) {
            MGlobal::displayInfo("Checking Zero UV faces");
        }
        status = findZeroUvFaces();
        CHECK_MSTATUS_AND_RETURN_IT(status);
        break;
    case UvChecker::UN_ASSIGNED_UVS:
        if (verbose) {
            MGlobal::displayInfo("Checking UnassignedUVs");
        }
        bool result;
        result = hasUnassignedUVs();
        MPxCommand::setResult(result);
        break;
    case UvChecker::NEGATIVE_SPACE_UVS:
        if (verbose)
            MGlobal::displayInfo("Checking UVs in negative space");
        status = findNegativeSpaceUVs();
        CHECK_MSTATUS_AND_RETURN_IT(status);
        break;
    default:
        MGlobal::displayError("Invalid check number");
        return MS::kFailure;
        break;
    }

    return MS::kSuccess;
}

MStatus UvChecker::undoIt()
{
    return MS::kSuccess;
}

bool UvChecker::isUndoable() const
{
    return false;
}

void* UvChecker::creator()
{
    return new UvChecker;
}

MStatus UvChecker::findUdimIntersections()
{
    MStatus status;

    MIntArray indexArray;
    MFnMesh fnMesh(mDagPath);

    std::set<int> indexSet;

    for (MItMeshPolygon mItPoly(mDagPath); !mItPoly.isDone(); mItPoly.next()) {

        int vCount = mItPoly.polygonVertexCount();
        int currentUVindex;
        int nextUVindex;
        float u1, v1, u2, v2;

        for (int i = 0; i < vCount; i++) {
            mItPoly.getUVIndex(i, currentUVindex);

            if (i == vCount - 1) {
                mItPoly.getUVIndex(0, nextUVindex);
            }
            else {
                mItPoly.getUVIndex(i + 1, nextUVindex);
            }

            fnMesh.getUV(currentUVindex, u1, v1);
            fnMesh.getUV(nextUVindex, u2, v2);

            if (floor(u1) == floor(u2) && floor(v1) == floor(v2)) {
            }
            else {
                indexSet.insert(currentUVindex);
                indexSet.insert(nextUVindex);
            }
        }
    }

    std::set<int>::iterator indexSetIter;
    for (indexSetIter = indexSet.begin(); indexSetIter != indexSet.end(); ++indexSetIter) {
        indexArray.append(*indexSetIter);
    }

    unsigned int arrayLength = indexArray.length();
    MStringArray resultArray;
    for (unsigned int i = 0; i < arrayLength; i++) {
        MString index;
        index.set(indexArray[i]);
        MString s = mDagPath.fullPathName() + ".map[" + index + "]";
        resultArray.append(s);
    }
    MPxCommand::setResult(resultArray);

    return MS::kSuccess;
}

MStatus UvChecker::findNoUvFaces()
{
    MStringArray resultArray;

    bool hasUVs;
    for (MItMeshPolygon itPoly(mDagPath); !itPoly.isDone(); itPoly.next()) {
        hasUVs = itPoly.hasUVs();
        if (hasUVs == false) {
            MString index;
            index.set(itPoly.index());
            MString s = mDagPath.fullPathName() + ".f[" + index + "]";
            resultArray.append(s);
        }
    }
    MPxCommand::setResult(resultArray);
    return MS::kSuccess;
}

MStatus UvChecker::findZeroUvFaces()
{
    MStringArray resultArray;
    double area;
    bool hasUVs;
    MString index;

    MString temp;
    temp.set(minUVArea);
    MGlobal::displayInfo(temp);

    for (MItMeshPolygon itPoly(mDagPath); !itPoly.isDone(); itPoly.next()) {
        hasUVs = itPoly.hasUVs();
        if (hasUVs == false) {
        }
        else {
            itPoly.getUVArea(area);
            temp.set(area);
            // MGlobal::displayInfo(temp);
            if (area < minUVArea) {
                index.set(itPoly.index());
                MString s = mDagPath.fullPathName() + ".f[" + index + "]";
                resultArray.append(s);
            }
        }
    }
    MPxCommand::setResult(resultArray);
    return MS::kSuccess;
}

bool UvChecker::hasUnassignedUVs()
{
    MFnMesh fnMesh(mDagPath);
    int numUVs = fnMesh.numUVs();
    MIntArray uvCounts;
    MIntArray uvIds;
    fnMesh.getAssignedUVs(uvCounts, uvIds);
    unsigned int numUvIds = uvIds.length();

    std::vector<int> uvIdVec;
    uvIdVec.reserve(numUvIds);
    for (int i = 0; i < numUvIds; i++) {
        uvIdVec.push_back(uvIds[i]);
    }

    // Remove duplicate elements
    std::sort(uvIdVec.begin(), uvIdVec.end());
    uvIdVec.erase(std::unique(uvIdVec.begin(), uvIdVec.end()), uvIdVec.end());

    int numAssignedUVs = uvIdVec.size();

    if (numUVs != numAssignedUVs) {
        return true;
    } else {
        return false;
    }
}

MStatus UvChecker::findNegativeSpaceUVs()
{
    MFnMesh fnMesh(mDagPath);
    MFloatArray uArray, vArray;
    fnMesh.getUVs(uArray, vArray);

    std::vector<int> indices;

    int numUVs = fnMesh.numUVs();

    for (int i = 0; i < numUVs; i++) {
        float u = uArray[i];
        if (u < 0.0) {
            indices.push_back(i);
            continue;
        }
        float v = vArray[i];
        if (v < 0.0) {
            indices.push_back(i);
            continue;
        }
    }

        // Remove duplicate elements
    std::sort(indices.begin(), indices.end());
    indices.erase(std::unique(indices.begin(), indices.end()), indices.end());

    std::string path;
    MString mPath;
    MStringArray resultArray;
    for (size_t i = 0; i < indices.size(); i++) {
        path = mDagPath.fullPathName().asChar();
        path = path + ".map[" + std::to_string(indices[i]) + "]";
        mPath = path.c_str();
        resultArray.append(mPath);
    }

    MPxCommand::setResult(resultArray);
    return MS::kSuccess;
}