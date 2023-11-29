#pragma once

#include "CoreMinimal.h"
#include "Eigen/Dense"

namespace MotionEngine
{

    class Geometry
    {
        public:
            Geometry();
            ~Geometry();

            Geometry(const Geometry* GeometrySpecs);

            enum GeometryType
            {
                UNKNOWN,
                SPHERE,
                CYLINDER,
                BOX,
                MESH
            };

            GeometryType Type;
    };

    class GeometryMesh : public Geometry
    {
        public:
            GeometryMesh();
            ~GeometryMesh();

            GeometryMesh(const GeometryMesh* GeometryMeshSpecs);

            FString MeshDirectory;
            FString MeshFileName;
            Eigen::Vector3d MeshScale;

            void Clear(void);
    };

    class Visual
    {
        public:
            Visual();
            ~Visual();

            Visual(const Visual* VisualSpecs);

            TSharedPtr< Geometry > VisualGeometry;

            Eigen::Vector3d VisualOriginXYZ;
            Eigen::Vector3d VisualOriginRPY;

            Eigen::Matrix4d VisualOriginG;

            void Clear(void);
    };

    class Collision
    {
        public:
            Collision();
            ~Collision();

            Collision(const Collision* CollisionSpecs);

            void Clear(void);
    };

    class Inertial
    {
        public:
            Inertial();
            ~Inertial();

            Inertial(const Inertial* InertialSpecs);

            void Clear(void);
    };

}