#include "MotionObjectProperties.h"

namespace MotionEngine
{

Geometry::Geometry()
{
    Type = Geometry::UNKNOWN;
}

Geometry::~Geometry()
{

}

Geometry::Geometry(const Geometry* GeometrySpecs)
{
    if(GeometrySpecs)
    {
        Type = GeometrySpecs->Type;
    }
    else
    {
        Type = Geometry::UNKNOWN;
    }
}

GeometryMesh::GeometryMesh()
{
    Clear();

    Type = Geometry::MESH;
}

GeometryMesh::~GeometryMesh()
{

}

GeometryMesh::GeometryMesh(const GeometryMesh* GeometryMeshSpecs)
{
    if(GeometryMeshSpecs)
    {
        Type = GeometryMeshSpecs->Type;

        MeshDirectory = GeometryMeshSpecs->MeshDirectory;
        MeshFileName = GeometryMeshSpecs->MeshFileName;
        MeshScale = GeometryMeshSpecs->MeshScale;
    }
    else
    {
        Clear();

        Type = Geometry::MESH;
    }
}

void GeometryMesh::Clear()
{
    Type = Geometry::UNKNOWN;

    MeshDirectory.Reset(20);
    MeshFileName.Reset(20);

    MeshScale(0) = 1;
    MeshScale(1) = 1;
    MeshScale(2) = 1;
}

Visual::Visual()
{
    Clear();
}

Visual::~Visual()
{

}

Visual::Visual(const Visual* VisualSpecs)
{
    VisualGeometry.Reset();

    if(VisualSpecs)
    {
        if(VisualSpecs->VisualGeometry)
        {
            switch (VisualSpecs->VisualGeometry->Type)
            {
            case Geometry::SPHERE:

                break;
            case Geometry::CYLINDER:

                break;
            case Geometry::BOX:

                break;
            case Geometry::MESH:
                VisualGeometry = MakeShareable(new GeometryMesh(StaticCastSharedPtr<GeometryMesh>(VisualSpecs->VisualGeometry).Get()));
                break;
            default:
                VisualGeometry = MakeShareable(new Geometry());
                break;
            }
        }

        VisualOriginXYZ = VisualSpecs->VisualOriginXYZ;
        VisualOriginRPY = VisualSpecs->VisualOriginRPY;
        VisualOriginG = VisualSpecs->VisualOriginG;
    }
    else
    {
        Clear();
    }
}

void Visual::Clear()
{
    VisualGeometry.Reset();

    VisualOriginXYZ = Eigen::Vector3d::Zero();
    VisualOriginRPY = Eigen::Vector3d::Zero();

    VisualOriginG = Eigen::Matrix4d::Identity();
}

Collision::Collision()
{
    Clear();
}

Collision::~Collision()
{

}

Collision::Collision(const Collision* CollisionSpecs)
{

}

void Collision::Clear()
{

}

Inertial::Inertial()
{
    Clear();
}

Inertial::~Inertial()
{

}

Inertial::Inertial(const Inertial* InertialSpecs)
{

}

void Inertial::Clear()
{

}

}