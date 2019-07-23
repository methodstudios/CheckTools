#include <iostream>
#include <chrono>
#include <memory>
#include <vector>
#include <stdexcept>

#include <meshChecker.h>

#include <maya/MString.h>
#include <maya/MFileIO.h>
#include <maya/MDagPath.h>
#include <maya/MIntArray.h>
#include <maya/MGlobal.h>
#include <maya/MLibrary.h>
#include <maya/MItDag.h>
#include <maya/MFnMesh.h>


// MAYA_LOCATION must be set
struct MLibraryAutoScope
{
    explicit MLibraryAutoScope(const char* name)
            : status{MLibrary::initialize(name)}
    {
        if(!status)
        {
            throw std::runtime_error("Failed to initialize!");
        }
    }
    ~MLibraryAutoScope()
    {
        MLibrary::cleanup();
    }
    MStatus status;
};


int main(int argc, char** argv)
{
    if(argc != 2)
    {
        std::cout << "Invalid arguments [check] [path]" << std::endl;
        return 0;
    }

    MLibraryAutoScope lib_scope{argv[0]};

    MString file_name{argv[1]};
    if(!MFileIO::open(file_name))
    {
        std::cout << "file not found" << std::endl;
        return 0;
    }

    for(MItDag it{MItDag::kDepthFirst}; !it.isDone(); it.next())
    {
        MDagPath path;
        it.getPath(path);

        if(path.apiType() == MFn::kMesh)
        {
            std::cout << it.fullPathName() << std::endl;
            MFnMesh mesh{path};
            MIntArray indices;

            auto start = std::chrono::system_clock::now();

            // call op here

            auto end = std::chrono::system_clock::now();
            std::cout << std::chrono::duration<double>(end-start).count() << std::endl;
        }
    }

    return 0;
}