using System;

namespace PhysX
{
	// TODO needs enums and struct default values

	[CRepr] public struct PxAllocatorCallback
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxAssertHandler
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxVec3
	{
		public float x;
		public float y;
		public float z;
	}
	[CRepr] public struct PxQuat
	{
		public float x;
		public float y;
		public float z;
		public float w;
	}
	[CRepr] public struct PxMat33
	{
		public PxVec3 column0;
		public PxVec3 column1;
		public PxVec3 column2;
	}
	[CRepr] public struct PxPlane
	{
		public PxVec3 n;
		public float d;
	}
	[CRepr] public struct PxTransform
	{
		public PxQuat q;
		public PxVec3 p;
	}
	[CRepr] public struct PxVec4
	{
		public float x;
		public float y;
		public float z;
		public float w;
	}
	[CRepr] public struct PxMat44
	{
		public PxVec4 column0;
		public PxVec4 column1;
		public PxVec4 column2;
		public PxVec4 column3;
	}
	[CRepr] public struct PxBounds3
	{
		public PxVec3 minimum;
		public PxVec3 maximum;
	}
	[CRepr] public struct PxErrorCallback
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxInputStream
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxInputData
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxOutputStream
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxVec2
	{
		public float x;
		public float y;
	}
	[CRepr] public struct PxStridedData
	{
		public uint32 stride;
		public char8[4] structgen_pad0;
		public void* data;
	}
	[CRepr] public struct PxBoundedData
	{
		public uint32 stride;
		public char8[4] structgen_pad0;
		public void* data;
		public uint32 count;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxDebugPoint
	{
		public PxVec3 pos;
		public uint32 color;
	}
	[CRepr] public struct PxDebugLine
	{
		public PxVec3 pos0;
		public uint32 color0;
		public PxVec3 pos1;
		public uint32 color1;
	}
	[CRepr] public struct PxDebugTriangle
	{
		public PxVec3 pos0;
		public uint32 color0;
		public PxVec3 pos1;
		public uint32 color1;
		public PxVec3 pos2;
		public uint32 color2;
	}
	[CRepr] public struct PxDebugText
	{
		public PxVec3 position;
		public float size;
		public uint32 color;
		public char8[4] structgen_pad0;
		public char8* string;
	}
	[CRepr] public struct PxRenderBuffer
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxProcessPxBaseCallback
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxBaseFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxBase
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxSerializationContext
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxCollection
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxDeserializationContext
	{
		public char8[8] structgen_pad0;
		public uint8* mExtraDataAddress;
	}
	[CRepr] public struct PxSerializationRegistry
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxSerializer
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxRepXSerializer
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxTolerancesScale
	{
		public float length;
		public float speed;
	}
	[CRepr] public struct PxStringTable
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxFoundation
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxProfilerCallback
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxPhysicsInsertionCallback
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxTaskManager
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxCpuDispatcher
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxTask
	{
		public char8[8] structgen_pad0;
		public uint64 mContextID;
		public PxTaskManager* mTm;
		public uint32 mTaskID;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxBaseTask
	{
		public char8[8] structgen_pad0;
		public uint64 mContextID;
		public PxTaskManager* mTm;
	}
	[CRepr] public struct PxLightCpuTask
	{
		public char8[8] structgen_pad0;
		public uint64 mContextID;
		public PxTaskManager* mTm;
		public PxBaseTask* mCont;
		public int32 mRefCount;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxGeometry
	{
		public int32 mType;
	}
	[CRepr] public struct PxBoxGeometry
	{
		public int32 mType;
		public PxVec3 halfExtents;
	}
	[CRepr] public struct PxBVHStructure
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxCapsuleGeometry
	{
		public int32 mType;
		public float radius;
		public float halfHeight;
	}
	[CRepr] public struct PxConvexMesh
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxHullPolygon
	{
		public float[4] mPlane;
		public uint16 mNbVerts;
		public uint16 mIndexBase;
	}
	[CRepr] public struct PxMeshScale
	{
		public PxVec3 scale;
		public PxQuat rotation;
	}
	[CRepr] public struct PxConvexMeshGeometryFlags
	{
		public uint8 mBits;
	}
	[CRepr] public struct PxPadding_3__Pod
	{
		public uint8[3] mPadding;
	}
	[CRepr] public struct PxConvexMeshGeometry
	{
		public int32 mType;
		public PxMeshScale scale;
		public PxConvexMesh* convexMesh;
		public PxConvexMeshGeometryFlags meshFlags;
		public PxPadding_3__Pod paddingFromFlags;
		public char8[4] structgen_pad0;
	}
	[CRepr] public struct PxSphereGeometry
	{
		public int32 mType;
		public float radius;
	}
	[CRepr] public struct PxPlaneGeometry
	{
		public int32 mType;
	}
	[CRepr] public struct PxMeshGeometryFlags
	{
		public uint8 mBits;
	}
	[CRepr] public struct PxTriangleMesh
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxTriangleMeshGeometry
	{
		public int32 mType;
		public PxMeshScale scale;
		public PxMeshGeometryFlags meshFlags;
		public PxPadding_3__Pod paddingFromFlags;
		public char8[4] structgen_pad0;
		public PxTriangleMesh* triangleMesh;
	}
	[CRepr] public struct PxHeightField
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxHeightFieldGeometry
	{
		public int32 mType;
		public char8[4] structgen_pad0;
		public PxHeightField* heightField;
		public float heightScale;
		public float rowScale;
		public float columnScale;
		public PxMeshGeometryFlags heightFieldFlags;
		public PxPadding_3__Pod paddingFromFlags;
	}
	[CRepr] public struct Anonymous59_Pod
	{
		public uint8[4] geometry;
		public uint8[16] box_;
		public uint8[8] sphere;
		public uint8[12] capsule;
		public uint8[4] plane;
		public uint8[48] convex;
		public uint8[48] mesh;
		public uint8[32] heightfield;
	}
	[CRepr] public struct PxGeometryHolder
	{
		public char8[48] structgen_pad0;
	}
	[CRepr] public struct PxRigidActor
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxShape
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxActorShape
	{
		public PxRigidActor* actor;
		public PxShape* shape;
	}
	[CRepr] public struct PxQueryHit
	{
		public PxRigidActor* actor;
		public PxShape* shape;
		public uint32 faceIndex;
		public char8[4] structgen_pad0;
	}
	[CRepr] public struct PxHitFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxLocationHit
	{
		public PxRigidActor* actor;
		public PxShape* shape;
		public uint32 faceIndex;
		public char8[4] structgen_pad0;
		public PxHitFlags flags;
		public char8[2] structgen_pad1;
		public PxVec3 position;
		public PxVec3 normal;
		public float distance;
	}
	[CRepr] public struct PxRaycastHit
	{
		public PxRigidActor* actor;
		public PxShape* shape;
		public uint32 faceIndex;
		public char8[4] structgen_pad0;
		public PxHitFlags flags;
		public char8[2] structgen_pad1;
		public PxVec3 position;
		public PxVec3 normal;
		public float distance;
		public float u;
		public float v;
	}
	[CRepr] public struct PxSweepHit
	{
		public PxRigidActor* actor;
		public PxShape* shape;
		public uint32 faceIndex;
		public char8[4] structgen_pad0;
		public PxHitFlags flags;
		public char8[2] structgen_pad1;
		public PxVec3 position;
		public PxVec3 normal;
		public float distance;
		public uint32 padTo16Bytes;
		public char8[4] structgen_pad2;
	}
	[CRepr] public struct PxBitAndByte
	{
		public uint8 mData;
	}
	[CRepr] public struct PxHeightFieldSample
	{
		public int16 height;
		public PxBitAndByte materialIndex0;
		public PxBitAndByte materialIndex1;
	}
	[CRepr] public struct PxHeightFieldFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxHeightFieldDesc
	{
		public uint32 nbRows;
		public uint32 nbColumns;
		public uint32 format;
		public char8[4] structgen_pad0;
		public PxStridedData samples;
		public float convexEdgeThreshold;
		public PxHeightFieldFlags flags;
		public char8[2] structgen_pad1;
	}
	[CRepr] public struct PxTriangle
	{
		public PxVec3[3] verts;
	}
	[CRepr] public struct PxMeshFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxSimpleTriangleMesh
	{
		public PxBoundedData points;
		public PxBoundedData triangles;
		public PxMeshFlags flags;
		public char8[6] structgen_pad0;
	}
	[CRepr] public struct PxTriangleMeshFlags
	{
		public uint8 mBits;
	}
	[CRepr] public struct PxActor
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxScene
	{
		public char8[8] structgen_pad0;
		public void* userData;
	}
	[CRepr] public struct PxActorFlags
	{
		public uint8 mBits;
	}
	[CRepr] public struct PxAggregate
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxArticulationBase
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxArticulationLink
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxArticulationImpl;
	[CRepr] public struct PxArticulationJointBase
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxArticulation
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxArticulationDriveCache
	{
		public char8[1] structgen_pad0;
	}
	[CRepr] public struct PxConstraintInvMassScale
	{
		public float linear0;
		public float angular0;
		public float linear1;
		public float angular1;
	}
	[CRepr] public struct PxConstraintVisualizer
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxConstraintConnector
	{
		// public void* vtable_;
	}
	[CRepr] public struct pvdsdk_PvdDataStream;
	[CRepr] public struct PxConstraint
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxSolverBody
	{
		public PxVec3 linearVelocity;
		public uint16 maxSolverNormalProgress;
		public uint16 maxSolverFrictionProgress;
		public PxVec3 angularState;
		public uint32 solverProgress;
	}
	[CRepr] public struct PxSolverBodyData
	{
		public PxVec3 linearVelocity;
		public float invMass;
		public PxVec3 angularVelocity;
		public float reportThreshold;
		public PxMat33 sqrtInvInertia;
		public float penBiasClamp;
		public uint32 nodeIndex;
		public float maxContactImpulse;
		public PxTransform body2World;
		public uint16 lockFlags;
		public uint16 pad;
	}
	[CRepr] public struct PxConstraintAllocator
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxTGSSolverBodyVel
	{
		public PxVec3 linearVelocity;
		public uint16 nbStaticInteractions;
		public uint16 maxDynamicPartition;
		public PxVec3 angularVelocity;
		public uint32 partitionMask;
		public PxVec3 deltaAngDt;
		public float maxAngVel;
		public PxVec3 deltaLinDt;
		public uint16 lockFlags;
		public bool isKinematic;
		public uint8 pad;
	}
	[CRepr] public struct PxTGSSolverBodyData
	{
		public PxVec3 originalLinearVelocity;
		public float maxContactImpulse;
		public PxVec3 originalAngularVelocity;
		public float penBiasClamp;
		public float invMass;
		public uint32 nodeIndex;
		public float reportThreshold;
		public uint32 pad;
	}
	[CRepr] public struct PxSpatialForce
	{
		public PxVec3 force;
		public float pad0;
		public PxVec3 torque;
		public float pad1;
	}
	[CRepr] public struct PxArticulationRootLinkData
	{
		public PxTransform transform;
		public PxVec3 worldLinVel;
		public PxVec3 worldAngVel;
		public PxVec3 worldLinAccel;
		public PxVec3 worldAngAccel;
	}
	[CRepr] public struct PxArticulationCache
	{
		public PxSpatialForce* externalForces;
		public float* denseJacobian;
		public float* massMatrix;
		public float* jointVelocity;
		public float* jointAcceleration;
		public float* jointPosition;
		public float* jointForce;
		public char8[16] structgen_pad0;
		public PxArticulationRootLinkData* rootLinkData;
		public float* coefficientMatrix;
		public float* lambda;
		public void* scratchMemory;
		public void* scratchAllocator;
		public uint32 version;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxArticulationReducedCoordinate
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxArticulationFlags
	{
		public uint8 mBits;
	}
	[CRepr] public struct PxArticulationCacheFlags
	{
		public uint8 mBits;
	}
	[CRepr] public struct PxJoint
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxArticulationJointImpl;
	[CRepr] public struct PxArticulationJoint
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxArticulationJointReducedCoordinate
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxFilterData
	{
		public uint32 word0;
		public uint32 word1;
		public uint32 word2;
		public uint32 word3;
	}
	[CRepr] public struct PxMaterial
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	public enum PxShapeFlags : uint8
	{
		/**
		\brief The shape will partake in collision in the physical simulation.

		\note It is illegal to raise the eSIMULATION_SHAPE and eTRIGGER_SHAPE flags.
		In the event that one of these flags is already raised the sdk will reject any 
		attempt to raise the other.  To raise the eSIMULATION_SHAPE first ensure that 
		eTRIGGER_SHAPE is already lowered.

		\note This flag has no effect if simulation is disabled for the corresponding actor (see
		#PxActorFlag::eDISABLE_SIMULATION).

		@see PxSimulationEventCallback.onContact() PxScene.setSimulationEventCallback() PxShape.setFlag(),
		PxShape.setFlags()
		*/
		eSIMULATION_SHAPE = (1 << 0),

		/**
		\brief The shape will partake in scene queries (ray casts, overlap tests, sweeps, ...).
		*/
		eSCENE_QUERY_SHAPE = (1 << 1),

		/**
		\brief The shape is a trigger which can send reports whenever other shapes enter/leave its volume.

		\note Triangle meshes and heightfields can not be triggers. Shape creation will fail in these cases.

		\note Shapes marked as triggers do not collide with other objects. If an object should act both
		as a trigger shape and a collision shape then create a rigid body with two shapes, one being a 
		trigger shape and the other a collision shape. 	It is illegal to raise the eTRIGGER_SHAPE and 
		eSIMULATION_SHAPE flags on a single PxShape instance.  In the event that one of these flags is already 
		raised the sdk will reject any attempt to raise the other.  To raise the eTRIGGER_SHAPE flag first 
		ensure that eSIMULATION_SHAPE flag is already lowered.

		\note Trigger shapes will no longer send notification events for interactions with other trigger shapes.

		\note Shapes marked as triggers are allowed to participate in scene queries, provided the eSCENE_QUERY_SHAPE
		flag is set. 

		\note This flag has no effect if simulation is disabled for the corresponding actor (see
		#PxActorFlag::eDISABLE_SIMULATION).

		@see PxSimulationEventCallback.onTrigger() PxScene.setSimulationEventCallback() PxShape.setFlag(),
		PxShape.setFlags()
		*/
		eTRIGGER_SHAPE = (1 << 2),

		/**
		\brief Enable debug renderer for this shape

		@see PxScene.getRenderBuffer() PxRenderBuffer PxVisualizationParameter
		*/
		eVISUALIZATION = (1 << 3) }
	[CRepr] public struct PxRigidBody
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxRigidBodyFlags
	{
		public uint8 mBits;
	}
	[CRepr] public struct PxFilterFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxSimulationFilterCallback
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxPairFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxQueryFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxQueryFilterData
	{
		public PxFilterData data;
		public PxQueryFlags flags;
		public char8[2] structgen_pad0;
	}
	[CRepr] public struct PxQueryFilterCallback
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxRaycastQueryResult;
	[CRepr] public struct PxSweepQueryResult;
	[CRepr] public struct PxOverlapQueryResult;
	[CRepr] public struct PxOverlapHit
	{
		public PxRigidActor* actor;
		public PxShape* shape;
		public uint32 faceIndex;
		public char8[4] structgen_pad0;
		public uint32 padTo16Bytes;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxBatchQueryMemory
	{
		public PxRaycastQueryResult* userRaycastResultBuffer;
		public PxRaycastHit* userRaycastTouchBuffer;
		public PxSweepQueryResult* userSweepResultBuffer;
		public PxSweepHit* userSweepTouchBuffer;
		public PxOverlapQueryResult* userOverlapResultBuffer;
		public PxOverlapHit* userOverlapTouchBuffer;
		public uint32 raycastTouchBufferSize;
		public uint32 sweepTouchBufferSize;
		public uint32 overlapTouchBufferSize;
		public uint32 raycastResultBufferSize;
		public uint32 sweepResultBufferSize;
		public uint32 overlapResultBufferSize;
	}
	[CRepr] public struct PxBatchQueryDesc
	{
		public void* filterShaderData;
		public uint32 filterShaderDataSize;
		public char8[4] structgen_pad0;
		public void* preFilterShader;
		public void* postFilterShader;
		public PxBatchQueryMemory queryMemory;
	}
	[CRepr] public struct PxBatchQuery
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxQueryCache
	{
		public PxShape* shape;
		public PxRigidActor* actor;
		public uint32 faceIndex;
		public char8[4] structgen_pad0;
	}
	[CRepr] public struct PxConstraintFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxConstraintShaderTable
	{
		public void* solverPrep;
		public void* project;
		public void* visualize;
		public uint32 flag;
		public char8[4] structgen_pad0;
	}
	[CRepr] public struct PxMassModificationProps
	{
		public float mInvMassScale0;
		public float mInvInertiaScale0;
		public float mInvMassScale1;
		public float mInvInertiaScale1;
	}
	[CRepr] public struct PxContactPatch
	{
		public PxMassModificationProps mMassModification;
		public PxVec3 normal;
		public float restitution;
		public float dynamicFriction;
		public float staticFriction;
		public uint8 startContactIndex;
		public uint8 nbContacts;
		public uint8 materialFlags;
		public uint8 internalFlags;
		public uint16 materialIndex0;
		public uint16 materialIndex1;
	}
	[CRepr] public struct PxContact
	{
		public PxVec3 contact;
		public float separation;
	}
	[CRepr] public struct PxContactStreamIterator
	{
		public PxVec3 zero;
		public char8[4] structgen_pad0;
		public PxContactPatch* patch;
		public PxContact* contact;
		public uint32* faceIndice;
		public uint32 totalPatches;
		public uint32 totalContacts;
		public uint32 nextContactIndex;
		public uint32 nextPatchIndex;
		public uint32 contactPatchHeaderSize;
		public uint32 contactPointSize;
		public uint32 mStreamFormat;
		public uint32 forceNoResponse;
		public bool pointStepped;
		public char8[3] structgen_pad1;
		public uint32 hasFaceIndices;
	}
	[CRepr] public struct PxModifiableContact
	{
		public PxVec3 contact;
		public float separation;
		public PxVec3 targetVelocity;
		public float maxImpulse;
		public PxVec3 normal;
		public float restitution;
		public uint32 materialFlags;
		public uint16 materialIndex0;
		public uint16 materialIndex1;
		public float staticFriction;
		public float dynamicFriction;
	}
	[CRepr] public struct PxContactSet
	{
		public uint32 mCount;
		public char8[4] structgen_pad0;
		public PxModifiableContact* mContacts;
	}
	[CRepr] public struct PxContactModifyCallback
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxContactModifyPair
	{
		public PxRigidActor*[2] actor;
		public PxShape*[2] shape;
		public PxTransform[2] transform;
		public PxContactSet contacts;
	}
	[CRepr] public struct PxCCDContactModifyCallback
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxDeletionListener
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxDataAccessFlags
	{
		public uint8 mBits;
	}
	[CRepr] public struct PxLockedData
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxMaterialFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxPhysics
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxSimulationEventCallback
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxBroadPhaseCallback
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxSceneLimits
	{
		public uint32 maxNbActors;
		public uint32 maxNbBodies;
		public uint32 maxNbStaticShapes;
		public uint32 maxNbDynamicShapes;
		public uint32 maxNbAggregates;
		public uint32 maxNbConstraints;
		public uint32 maxNbRegions;
		public uint32 maxNbBroadPhaseOverlaps;
	}
	public enum PxSceneFlags : uint32
	{
		/**
		\brief Enable Active Actors Notification.

		This flag enables the Active Actor Notification feature for a scene.  This
		feature defaults to disabled.  When disabled, the function
		PxScene::getActiveActors() will always return a NULL list.

		\note There may be a performance penalty for enabling the Active Actor Notification, hence this flag should
		only be enabled if the application intends to use the feature.

		<b>Default:</b> False
		*/
		eENABLE_ACTIVE_ACTORS = (1 << 0),

		/**
		\brief Enables a second broad phase check after integration that makes it possible to prevent objects from
		tunneling through eachother.

		PxPairFlag::eDETECT_CCD_CONTACT requires this flag to be specified.

		\note For this feature to be effective for bodies that can move at a significant velocity, the user should raise
		the flag PxRigidBodyFlag::eENABLE_CCD for them. \note This flag is not mutable, and must be set in PxSceneDesc
		at scene creation.

		<b>Default:</b> False

		@see PxRigidBodyFlag::eENABLE_CCD, PxPairFlag::eDETECT_CCD_CONTACT, eDISABLE_CCD_RESWEEP
		*/
		eENABLE_CCD = (1 << 1),

		/**
		\brief Enables a simplified swept integration strategy, which sacrifices some accuracy for improved performance.

		This simplified swept integration approach makes certain assumptions about the motion of objects that are not
		made when using a full swept integration.  These assumptions usually hold but there are cases where they could
		result in incorrect behavior between a set of fast-moving rigid bodies. A key issue is that fast-moving dynamic
		objects may tunnel through each-other after a rebound. This will not happen if this mode is disabled. However,
		this approach will be potentially  faster than a full swept integration because it will perform significantly
		fewer sweeps in non-trivial scenes involving many fast-moving objects. This approach  should successfully resist
		objects passing through the static environment.

		PxPairFlag::eDETECT_CCD_CONTACT requires this flag to be specified.

		\note This scene flag requires eENABLE_CCD to be enabled as well. If it is not, this scene flag will do nothing.
		\note For this feature to be effective for bodies that can move at a significant velocity, the user should raise
		the flag PxRigidBodyFlag::eENABLE_CCD for them. \note This flag is not mutable, and must be set in PxSceneDesc
		at scene creation.

		<b>Default:</b> False

		@see PxRigidBodyFlag::eENABLE_CCD, PxPairFlag::eDETECT_CCD_CONTACT, eENABLE_CCD
		*/
		eDISABLE_CCD_RESWEEP = (1 << 2),

		/**
		\brief Enable adaptive forces to accelerate convergence of the solver. 
		
		\note This flag is not mutable, and must be set in PxSceneDesc at scene creation.

		<b>Default:</b> false
		*/
		eADAPTIVE_FORCE = (1 << 3),

		/**
		\brief Enable GJK-based distance collision detection system.
		
		\note This flag is not mutable, and must be set in PxSceneDesc at scene creation.

		<b>Default:</b> true
		*/
		eENABLE_PCM = (1 << 6),

		/**
		\brief Disable contact report buffer resize. Once the contact buffer is full, the rest of the contact reports
		will  not be buffered and sent.

		\note This flag is not mutable, and must be set in PxSceneDesc at scene creation.
		
		<b>Default:</b> false
		*/
		eDISABLE_CONTACT_REPORT_BUFFER_RESIZE = (1 << 7),

		/**
		\brief Disable contact cache.
		
		Contact caches are used internally to provide faster contact generation. You can disable all contact caches
		if memory usage for this feature becomes too high.

		\note This flag is not mutable, and must be set in PxSceneDesc at scene creation.

		<b>Default:</b> false
		*/
		eDISABLE_CONTACT_CACHE = (1 << 8),

		/**
		\brief Require scene-level locking

		When set to true this requires that threads accessing the PxScene use the
		multi-threaded lock methods.
		
		\note This flag is not mutable, and must be set in PxSceneDesc at scene creation.

		@see PxScene::lockRead
		@see PxScene::unlockRead
		@see PxScene::lockWrite
		@see PxScene::unlockWrite
		
		<b>Default:</b> false
		*/
		eREQUIRE_RW_LOCK = (1 << 9),

		/**
		\brief Enables additional stabilization pass in solver

		When set to true, this enables additional stabilization processing to improve that stability of complex
		interactions between large numbers of bodies.

		Note that this flag is not mutable and must be set in PxSceneDesc at scene creation. Also, this is an
		experimental feature which does result in some loss of momentum.
		*/
		eENABLE_STABILIZATION = (1 << 10),

		/**
		\brief Enables average points in contact manifolds

		When set to true, this enables additional contacts to be generated per manifold to represent the average point
		in a manifold. This can stabilize stacking when only a small number of solver iterations is used.

		Note that this flag is not mutable and must be set in PxSceneDesc at scene creation.
		*/
		eENABLE_AVERAGE_POINT = (1 << 11),

		/**
		\brief Do not report kinematics in list of active actors.

		Since the target pose for kinematics is set by the user, an application can track the activity state directly
		and use this flag to avoid that kinematics get added to the list of active actors.

		\note This flag has only an effect in combination with eENABLE_ACTIVE_ACTORS.

		@see eENABLE_ACTIVE_ACTORS

		<b>Default:</b> false
		*/
		eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS = (1 << 12),

		/*\brief Enables the GPU dynamics pipeline

		When set to true, a CUDA ARCH 3.0 or above-enabled NVIDIA GPU is present and the CUDA context manager has been
		configured, this will run the GPU dynamics pipelin instead of the CPU dynamics pipeline.

		Note that this flag is not mutable and must be set in PxSceneDesc at scene creation.
		*/
		eENABLE_GPU_DYNAMICS = (1 << 13),

		/**
		\brief Provides improved determinism at the expense of performance. 

		By default, PhysX provides limited determinism guarantees. Specifically, PhysX guarantees that the exact scene
		(same actors created in the same order) and simulated using the same  time-stepping scheme should provide the
		exact same behaviour.

		However, if additional actors are added to the simulation, this can affect the behaviour of the existing actors
		in the simulation, even if the set of new actors do not interact with  the existing actors.

		This flag provides an additional level of determinism that guarantees that the simulation will not change if
		additional actors are added to the simulation, provided those actors do not interfere with the existing actors
		in the scene. Determinism is only guaranteed if the actors are inserted in a consistent order each run in a
		newly-created scene and simulated using a consistent time-stepping scheme.

		Note that this flag is not mutable and must be set at scene creation.

		Note that enabling this flag can have a negative impact on performance.

		Note that this feature is not currently supported on GPU.

		<b>Default</b> false
		*/
		eENABLE_ENHANCED_DETERMINISM = (1 << 14),

		/**
		\brief Controls processing friction in all solver iterations

		By default, PhysX processes friction only in the final 3 position iterations, and all velocity
		iterations. This flag enables friction processing in all position and velocity iterations.

		The default behaviour provides a good trade-off between performance and stability and is aimed
		primarily at game development.

		When simulating more complex frictional behaviour, such as grasping of complex geometries with
		a robotic manipulator, better results can be achieved by enabling friction in all solver iterations.

		\note This flag only has effect with the default solver. The TGS solver always performs friction per-iteration.
		*/
		eENABLE_FRICTION_EVERY_ITERATION = (1 << 15),

		eMUTABLE_FLAGS = eENABLE_ACTIVE_ACTORS | eEXCLUDE_KINEMATICS_FROM_ACTIVE_ACTORS
	}
	[CRepr] public struct PxCudaContextManager;
	[CRepr] public struct PxgDynamicsMemoryConfig
	{
		public uint32 constraintBufferCapacity;
		public uint32 contactBufferCapacity;
		public uint32 tempBufferCapacity;
		public uint32 contactStreamSize;
		public uint32 patchStreamSize;
		public uint32 forceStreamCapacity;
		public uint32 heapCapacity;
		public uint32 foundLostPairsCapacity;
	}
	[CRepr] public struct PxSceneDesc
	{
		public PxVec3 gravity;
		public char8[4] structgen_pad0;
		public PxSimulationEventCallback* simulationEventCallback;
		public PxContactModifyCallback* contactModifyCallback;
		public PxCCDContactModifyCallback* ccdContactModifyCallback;
		public void* filterShaderData;
		public uint32 filterShaderDataSize;
		public char8[4] structgen_pad1;
		public void* filterShader;
		public PxSimulationFilterCallback* filterCallback;
		public uint32 kineKineFilteringMode = 1;
		public uint32 staticKineFilteringMode = 1;
		public uint32 broadPhaseType = 2;
		public char8[4] structgen_pad2;
		public PxBroadPhaseCallback* broadPhaseCallback;
		public PxSceneLimits limits;
		public uint32 frictionType;
		public uint32 solverType;
		public float bounceThresholdVelocity = 2.0f;
		public float frictionOffsetThreshold = 0.04f;
		public float ccdMaxSeparation = 0.04f;
		public float solverOffsetSlop;
		public PxSceneFlags flags = .eENABLE_PCM;
		public char8[4] structgen_pad3;
		public PxCpuDispatcher* cpuDispatcher;
		public PxCudaContextManager* cudaContextManager;
		public uint32 staticStructure = 1;
		public uint32 dynamicStructure = 1;
		public uint32 dynamicTreeRebuildRateHint = 100;
		public uint32 sceneQueryUpdateMode;
		public void* userData;
		public uint32 solverBatchSize = 128;
		public uint32 solverArticulationBatchSize = 16;
		public uint32 nbContactDataBlocks;
		public uint32 maxNbContactDataBlocks = 0xffff;
		public float maxBiasCoefficient = 3.40282347e+38f;
		public uint32 contactReportStreamBufferSize = 8192;
		public uint32 ccdMaxPasses = 1;
		public float ccdThreshold = 3.40282347e+38f;
		public float wakeCounterResetValue = 0.4f;
		public PxBounds3 sanityBounds = PxBounds3() { minimum = PxVec3() { x = -8.50705867e+37f, y = -8.50705867e+37f, z = -8.50705867e+37f }, maximum = PxVec3() { x = 8.50705867e+37f, y = 8.50705867e+37f, z = 8.50705867e+37f } };
		public PxgDynamicsMemoryConfig gpuDynamicsConfig = PxgDynamicsMemoryConfig() { constraintBufferCapacity = 33554432, contactBufferCapacity = 25165824, tempBufferCapacity = 16777216, contactStreamSize = 524288, patchStreamSize = 81920, forceStreamCapacity = 1048576, heapCapacity = 67108864, foundLostPairsCapacity = 262144 };
		public uint32 gpuMaxNumPartitions = 8;
		public uint32 gpuComputeVersion;
		public PxTolerancesScale tolerancesScale;
	}
	[CRepr] public struct PxRigidStatic
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxRigidDynamic
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxPruningStructure
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxDeletionEventFlags
	{
		public uint8 mBits;
	}
	[CRepr] public struct PxPvd
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxRigidDynamicLockFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxSimulationStatistics
	{
		public uint32 nbActiveConstraints;
		public uint32 nbActiveDynamicBodies;
		public uint32 nbActiveKinematicBodies;
		public uint32 nbStaticBodies;
		public uint32 nbDynamicBodies;
		public uint32 nbKinematicBodies;
		public uint32[7] nbShapes;
		public uint32 nbAggregates;
		public uint32 nbArticulations;
		public uint32 nbAxisSolverConstraints;
		public uint32 compressedContactSize;
		public uint32 requiredContactConstraintMemory;
		public uint32 peakConstraintMemory;
		public uint32 nbDiscreteContactPairsTotal;
		public uint32 nbDiscreteContactPairsWithCacheHits;
		public uint32 nbDiscreteContactPairsWithContacts;
		public uint32 nbNewPairs;
		public uint32 nbLostPairs;
		public uint32 nbNewTouches;
		public uint32 nbLostTouches;
		public uint32 nbPartitions;
		public uint32 nbBroadPhaseAdds;
		public uint32 nbBroadPhaseRemoves;
		public uint32[7] nbDiscreteContactPairs;
		public uint32[7] nbCCDPairs;
		public uint32[7] nbModifiedContactPairs;
		public uint32[7] nbTriggerPairs;
	}
	[CRepr] public struct PxPvdSceneClient
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxPvdSceneFlags
	{
		public uint8 mBits;
	}
	[CRepr] public struct pvdsdk_PvdDebugPoint;
	[CRepr] public struct pvdsdk_PvdDebugLine;
	[CRepr] public struct pvdsdk_PvdDebugTriangle;
	[CRepr] public struct pvdsdk_PvdDebugText;
	[CRepr] public struct pvdsdk_PvdClient;
	[CRepr] public struct PxDominanceGroupPair
	{
		public uint8 dominance0;
		public uint8 dominance1;
	}
	[CRepr] public struct PxActorTypeFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxContactPairHeaderFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxContactPairFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxContactPair
	{
		public PxShape*[2] shapes;
		public uint8* contactPatches;
		public uint8* contactPoints;
		public float* contactImpulses;
		public uint32 requiredBufferSize;
		public uint8 contactCount;
		public uint8 patchCount;
		public uint16 contactStreamSize;
		public PxContactPairFlags flags;
		public PxPairFlags events;
		public uint32[2] internalData;
		public char8[4] structgen_pad0;
	}
	[CRepr] public struct PxContactPairHeader
	{
		public PxRigidActor*[2] actors;
		public uint8* extraDataStream;
		public uint16 extraDataStreamSize;
		public PxContactPairHeaderFlags flags;
		public char8[4] structgen_pad0;
		public PxContactPair* pairs;
		public uint32 nbPairs;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxRaycastCallback
	{
		// public void* vtable_;
		public PxRaycastHit block;
		public bool hasBlock;
		public PxRaycastHit* touches;
		public uint32 maxNbTouches;
		public uint32 nbTouches;
	}
	[CRepr] public struct PxSweepCallback
	{
		// public void* vtable_;
		public PxSweepHit block;
		public bool hasBlock;
		public PxSweepHit* touches;
		public uint32 maxNbTouches;
		public uint32 nbTouches;
	}
	[CRepr] public struct PxOverlapCallback
	{
		// public void* vtable_;
		public PxOverlapHit block;
		public bool hasBlock;
		public PxOverlapHit* touches;
		public uint32 maxNbTouches;
		public uint32 nbTouches;
	}
	[CRepr] public struct PxBroadPhaseCaps
	{
		public uint32 maxNbRegions;
		public uint32 maxNbObjects;
		public bool needsPredefinedBounds;
		public char8[3] structgen_pad0;
	}
	[CRepr] public struct PxBroadPhaseRegion
	{
		public PxBounds3 bounds;
		public void* userData;
	}
	[CRepr] public struct PxBroadPhaseRegionInfo
	{
		public PxBroadPhaseRegion region;
		public uint32 nbStaticObjects;
		public uint32 nbDynamicObjects;
		public bool active;
		public bool overlap;
		public char8[6] structgen_pad0;
	}
	[CRepr] public struct PxSceneReadLock
	{
		public char8[8] structgen_pad0;
	}
	[CRepr] public struct PxSceneWriteLock
	{
		public char8[8] structgen_pad0;
	}
	[CRepr] public struct PxContactPairExtraDataItem
	{
		public uint8 type;
	}
	[CRepr] public struct PxContactPairVelocity
	{
		public uint8 type;
		public char8[3] structgen_pad0;
		public PxVec3[2] linearVelocity;
		public PxVec3[2] angularVelocity;
	}
	[CRepr] public struct PxContactPairPose
	{
		public uint8 type;
		public char8[3] structgen_pad0;
		public PxTransform[2] globalPose;
	}
	[CRepr] public struct PxContactPairIndex
	{
		public uint8 type;
		public char8[1] structgen_pad0;
		public uint16 index;
	}
	[CRepr] public struct PxContactPairExtraDataIterator
	{
		public uint8* currPtr;
		public uint8* endPtr;
		public PxContactPairVelocity* preSolverVelocity;
		public PxContactPairVelocity* postSolverVelocity;
		public PxContactPairPose* eventPose;
		public uint32 contactPairIndex;
		public char8[4] structgen_pad0;
	}
	[CRepr] public struct PxContactPairPoint
	{
		public PxVec3 position;
		public float separation;
		public PxVec3 normal;
		public uint32 internalFaceIndex0;
		public PxVec3 impulse;
		public uint32 internalFaceIndex1;
	}
	[CRepr] public struct PxTriggerPairFlags
	{
		public uint8 mBits;
	}
	[CRepr] public struct PxTriggerPair
	{
		public PxShape* triggerShape;
		public PxRigidActor* triggerActor;
		public PxShape* otherShape;
		public PxRigidActor* otherActor;
		public uint32 status;
		public PxTriggerPairFlags flags;
		public char8[3] structgen_pad0;
	}
	[CRepr] public struct PxConstraintInfo
	{
		public PxConstraint* constraint;
		public void* externalReference;
		public uint32 type;
		public char8[4] structgen_pad0;
	}
	[CRepr] public struct PxExtendedVec3
	{
		public double x;
		public double y;
		public double z;
	}
	[CRepr] public struct PxObstacle
	{
		public int32 mType;
		public char8[4] structgen_pad0;
		public void* mUserData;
		public PxExtendedVec3 mPos;
		public PxQuat mRot;
	}
	[CRepr] public struct PxBoxObstacle
	{
		public int32 mType;
		public char8[4] structgen_pad0;
		public void* mUserData;
		public PxExtendedVec3 mPos;
		public PxQuat mRot;
		public PxVec3 mHalfExtents;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxCapsuleObstacle
	{
		public int32 mType;
		public char8[4] structgen_pad0;
		public void* mUserData;
		public PxExtendedVec3 mPos;
		public PxQuat mRot;
		public float mHalfHeight;
		public float mRadius;
	}
	[CRepr] public struct PxObstacleContext
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxControllerManager
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxUserControllerHitReport
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxController
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxControllerShapeHit
	{
		public PxController* controller;
		public PxExtendedVec3 worldPos;
		public PxVec3 worldNormal;
		public PxVec3 dir;
		public float length;
		public char8[4] structgen_pad0;
		public PxShape* shape;
		public PxRigidActor* actor;
		public uint32 triangleIndex;
		public char8[4] structgen_pad1;
	}
	[CRepr] public struct PxControllersHit
	{
		public PxController* controller;
		public PxExtendedVec3 worldPos;
		public PxVec3 worldNormal;
		public PxVec3 dir;
		public float length;
		public char8[4] structgen_pad0;
		public PxController* other;
	}
	[CRepr] public struct PxControllerObstacleHit
	{
		public PxController* controller;
		public PxExtendedVec3 worldPos;
		public PxVec3 worldNormal;
		public PxVec3 dir;
		public float length;
		public char8[4] structgen_pad0;
		public void* userData;
	}
	[CRepr] public struct PxControllerFilterCallback
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxControllerFilters
	{
		public PxFilterData* mFilterData;
		public PxQueryFilterCallback* mFilterCallback;
		public PxQueryFlags mFilterFlags;
		public char8[6] structgen_pad0;
		public PxControllerFilterCallback* mCCTFilterCallback;
	}
	[CRepr] public struct PxControllerBehaviorCallback
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxControllerDesc
	{
		public char8[8] structgen_pad0;
		public PxExtendedVec3 position;
		public PxVec3 upDirection;
		public float slopeLimit;
		public float invisibleWallHeight;
		public float maxJumpHeight;
		public float contactOffset;
		public float stepOffset;
		public float density;
		public float scaleCoeff;
		public float volumeGrowth;
		public char8[4] structgen_pad1;
		public PxUserControllerHitReport* reportCallback;
		public PxControllerBehaviorCallback* behaviorCallback;
		public uint32 nonWalkableMode;
		public char8[4] structgen_pad2;
		public PxMaterial* material;
		public bool registerDeletionListener;
		public char8[7] structgen_pad3;
		public void* userData;
		public uint32 mType;
		public char8[4] structgen_pad4;
	}
	[CRepr] public struct PxControllerCollisionFlags
	{
		public uint8 mBits;
	}
	[CRepr] public struct PxControllerState
	{
		public PxVec3 deltaXP;
		public char8[4] structgen_pad0;
		public PxShape* touchedShape;
		public PxRigidActor* touchedActor;
		public uint32 touchedObstacleHandle;
		public uint32 collisionFlags;
		public bool standOnAnotherCCT;
		public bool standOnObstacle;
		public bool isMovingUp;
		public char8[5] structgen_pad1;
	}
	[CRepr] public struct PxControllerStats
	{
		public uint16 nbIterations;
		public uint16 nbFullUpdates;
		public uint16 nbPartialUpdates;
		public uint16 nbTessellation;
	}
	[CRepr] public struct PxBoxControllerDesc
	{
		public char8[8] structgen_pad0;
		public PxExtendedVec3 position;
		public PxVec3 upDirection;
		public float slopeLimit;
		public float invisibleWallHeight;
		public float maxJumpHeight;
		public float contactOffset;
		public float stepOffset;
		public float density;
		public float scaleCoeff;
		public float volumeGrowth;
		public char8[4] structgen_pad1;
		public PxUserControllerHitReport* reportCallback;
		public PxControllerBehaviorCallback* behaviorCallback;
		public uint32 nonWalkableMode;
		public char8[4] structgen_pad2;
		public PxMaterial* material;
		public bool registerDeletionListener;
		public char8[7] structgen_pad3;
		public void* userData;
		public uint32 mType;
		public char8[4] structgen_pad4;
		public float halfHeight;
		public float halfSideExtent;
		public float halfForwardExtent;
		public char8[4] structgen_pad5;
	}
	[CRepr] public struct PxBoxController
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxCapsuleControllerDesc
	{
		public char8[8] structgen_pad0;
		public PxExtendedVec3 position;
		public PxVec3 upDirection;
		public float slopeLimit;
		public float invisibleWallHeight;
		public float maxJumpHeight;
		public float contactOffset;
		public float stepOffset;
		public float density;
		public float scaleCoeff;
		public float volumeGrowth;
		public char8[4] structgen_pad1;
		public PxUserControllerHitReport* reportCallback;
		public PxControllerBehaviorCallback* behaviorCallback;
		public uint32 nonWalkableMode;
		public char8[4] structgen_pad2;
		public PxMaterial* material;
		public bool registerDeletionListener;
		public char8[7] structgen_pad3;
		public void* userData;
		public uint32 mType;
		public char8[4] structgen_pad4;
		public float radius;
		public float height;
		public uint32 climbingMode;
		public char8[4] structgen_pad5;
	}
	[CRepr] public struct PxCapsuleController
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxControllerBehaviorFlags
	{
		public uint8 mBits;
	}
	[CRepr] public struct PxControllerDebugRenderFlags
	{
		public uint32 mBits;
	}
	[CRepr] public struct PxConvexFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxConvexMeshDesc
	{
		public PxBoundedData points;
		public PxBoundedData polygons;
		public PxBoundedData indices;
		public PxConvexFlags flags;
		public uint16 vertexLimit;
		public uint16 quantizedCount;
		public char8[2] structgen_pad0;
	}
	[CRepr] public struct PxTypedStridedData_PxMaterialTableIndex_
	{
		public uint32 stride;
		public uint16* data;
	}
	[CRepr] public struct PxTriangleMeshDesc
	{
		public PxBoundedData points;
		public PxBoundedData triangles;
		public PxMeshFlags flags;
		public char8[6] structgen_pad0;
		public PxTypedStridedData_PxMaterialTableIndex_ materialIndices;
	}
	[CRepr] public struct PxBVH33MidphaseDesc
	{
		public float meshSizePerformanceTradeOff;
		public uint32 meshCookingHint;
	}
	[CRepr] public struct PxBVH34MidphaseDesc
	{
		public uint32 numPrimsPerLeaf;
	}
	[CRepr] public struct Anonymous216_Pod
	{
		public PxBVH33MidphaseDesc mBVH33Desc;
		public PxBVH34MidphaseDesc mBVH34Desc;
	}
	[CRepr] public struct PxMidphaseDesc
	{
		public char8[8] structgen_pad0;
		public uint32 mType;
	}
	[CRepr] public struct PxBVHStructureDesc
	{
		public PxBoundedData bounds;
	}
	[CRepr] public struct PxMeshPreprocessingFlags
	{
		public uint32 mBits;
	}
	[CRepr] public struct PxCookingParams
	{
		public float areaTestEpsilon;
		public float planeTolerance;
		public uint32 convexMeshCookingType;
		public bool suppressTriangleMeshRemapTable;
		public bool buildTriangleAdjacencies;
		public bool buildGPUData;
		public char8[1] structgen_pad0;
		public PxTolerancesScale scale;
		public PxMeshPreprocessingFlags meshPreprocessParams;
		public float meshWeldTolerance;
		public PxMidphaseDesc midphaseDesc;
		public uint32 gaussMapLimit;
	}
	[CRepr] public struct PxCooking
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxDefaultMemoryOutputStream
	{
		public char8[32] structgen_pad0;
	}
	[CRepr] public struct PxDefaultMemoryInputData
	{
		public char8[32] structgen_pad0;
	}
	[CRepr] public struct PxDefaultFileOutputStream
	{
		public char8[16] structgen_pad0;
	}
	[CRepr] public struct PxDefaultFileInputData
	{
		public char8[24] structgen_pad0;
	}
	[CRepr] public struct PxSpring
	{
		public float stiffness;
		public float damping;
	}
	[CRepr] public struct PxDistanceJoint
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxDistanceJointFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxDefaultAllocator
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxContactJoint
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxJacobianRow
	{
		public PxVec3 linear0;
		public PxVec3 linear1;
		public PxVec3 angular0;
		public PxVec3 angular1;
	}
	[CRepr] public struct PxFixedJoint
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxJointLimitParameters
	{
		public float restitution;
		public float bounceThreshold;
		public float stiffness;
		public float damping;
		public float contactDistance;
	}
	[CRepr] public struct PxJointLinearLimit
	{
		public float restitution;
		public float bounceThreshold;
		public float stiffness;
		public float damping;
		public float contactDistance;
		public float value;
	}
	[CRepr] public struct PxJointLinearLimitPair
	{
		public float restitution;
		public float bounceThreshold;
		public float stiffness;
		public float damping;
		public float contactDistance;
		public float upper;
		public float lower;
	}
	[CRepr] public struct PxJointAngularLimitPair
	{
		public float restitution;
		public float bounceThreshold;
		public float stiffness;
		public float damping;
		public float contactDistance;
		public float upper;
		public float lower;
	}
	[CRepr] public struct PxJointLimitCone
	{
		public float restitution;
		public float bounceThreshold;
		public float stiffness;
		public float damping;
		public float contactDistance;
		public float yAngle;
		public float zAngle;
	}
	[CRepr] public struct PxJointLimitPyramid
	{
		public float restitution;
		public float bounceThreshold;
		public float stiffness;
		public float damping;
		public float contactDistance;
		public float yAngleMin;
		public float yAngleMax;
		public float zAngleMin;
		public float zAngleMax;
	}
	[CRepr] public struct PxPrismaticJoint
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxPrismaticJointFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxRevoluteJoint
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxRevoluteJointFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxSphericalJoint
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxSphericalJointFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxD6Joint
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public void* userData;
	}
	[CRepr] public struct PxD6JointDriveFlags
	{
		public uint32 mBits;
	}
	[CRepr] public struct PxD6JointDrive
	{
		public float stiffness;
		public float damping;
		public float forceLimit;
		public PxD6JointDriveFlags flags;
	}
	[CRepr] public struct PxGroupsMask
	{
		public uint16 bits0;
		public uint16 bits1;
		public uint16 bits2;
		public uint16 bits3;
	}
	[CRepr] public struct PxDefaultErrorCallback
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxMassProperties
	{
		public PxMat33 inertiaTensor;
		public PxVec3 centerOfMass;
		public float mass;
	}
	[CRepr] public struct PxMeshOverlapUtil
	{
		public char8[1040] structgen_pad0;
	}
	[CRepr] public struct PxSerialization_PxXmlMiscParameter
	{
		public PxVec3 upVector;
		public PxTolerancesScale scale;
	}
	[CRepr] public struct PxBinaryConverter
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxDefaultCpuDispatcher
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxSceneQueryHit
	{
		public PxRigidActor* actor;
		public PxShape* shape;
		public uint32 faceIndex;
		public char8[4] structgen_pad0;
	}
	[CRepr] public struct PxSceneQueryFilterData
	{
		public PxFilterData data;
		public PxQueryFlags flags;
		public char8[2] structgen_pad0;
	}
	[CRepr] public struct PxSceneQueryFilterCallback
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxSceneQueryCache
	{
		public PxShape* shape;
		public PxRigidActor* actor;
		public uint32 faceIndex;
		public char8[4] structgen_pad0;
	}
	[CRepr] public struct PxSceneQueryFlags
	{
		public uint16 mBits;
	}
	[CRepr] public struct PxRepXObject
	{
		public char8* typeName;
		public void* serializable;
		public uint64 id;
	}
	[CRepr] public struct PxRepXInstantiationArgs
	{
		public char8[8] structgen_pad0;
		public PxCooking* cooker;
		public PxStringTable* stringTable;
	}
	[CRepr] public struct XmlWriter;
	[CRepr] public struct MemoryBuffer;
	[CRepr] public struct XmlReader;
	[CRepr] public struct XmlMemoryAllocator;
	[CRepr] public struct PxVehicleChassisData
	{
		public PxVec3 mMOI;
		public float mMass;
		public PxVec3 mCMOffset;
		public char8[4] structgen_pad0;
	}
	[CRepr] public struct PxFixedSizeLookupTable_eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES__Pod
	{
		public float[16] mDataPairs;
		public uint32 mNbDataPairs;
		public uint32[3] mPad;
	}
	[CRepr] public struct PxVehicleEngineData
	{
		public PxFixedSizeLookupTable_eMAX_NB_ENGINE_TORQUE_CURVE_ENTRIES__Pod mTorqueCurve;
		public float mMOI;
		public float mPeakTorque;
		public float mMaxOmega;
		public float mDampingRateFullThrottle;
		public float mDampingRateZeroThrottleClutchEngaged;
		public float mDampingRateZeroThrottleClutchDisengaged;
		public char8[8] structgen_pad0;
	}
	[CRepr] public struct PxVehicleGearsData
	{
		public float[32] mRatios;
		public float mFinalRatio;
		public uint32 mNbRatios;
		public float mSwitchTime;
		public char8[4] structgen_pad0;
	}
	[CRepr] public struct PxVehicleAutoBoxData
	{
		public float[32] mUpRatios;
		public float[32] mDownRatios;
	}
	[CRepr] public struct PxVehicleDifferential4WData
	{
		public float mFrontRearSplit;
		public float mFrontLeftRightSplit;
		public float mRearLeftRightSplit;
		public float mCentreBias;
		public float mFrontBias;
		public float mRearBias;
		public uint32 mType;
		public char8[4] structgen_pad0;
	}
	[CRepr] public struct PxVehicleDifferentialNWData
	{
		public char8[16] structgen_pad0;
	}
	[CRepr] public struct PxVehicleAckermannGeometryData
	{
		public float mAccuracy;
		public float mFrontWidth;
		public float mRearWidth;
		public float mAxleSeparation;
	}
	[CRepr] public struct PxVehicleClutchData
	{
		public float mStrength;
		public uint32 mAccuracyMode;
		public uint32 mEstimateIterations;
		public char8[4] structgen_pad0;
	}
	[CRepr] public struct PxVehicleTireLoadFilterData
	{
		public float mMinNormalisedLoad;
		public float mMinFilteredNormalisedLoad;
		public float mMaxNormalisedLoad;
		public float mMaxFilteredNormalisedLoad;
		public char8[16] structgen_pad0;
	}
	[CRepr] public struct PxVehicleWheelData
	{
		public float mRadius;
		public float mWidth;
		public float mMass;
		public float mMOI;
		public float mDampingRate;
		public float mMaxBrakeTorque;
		public float mMaxHandBrakeTorque;
		public float mMaxSteer;
		public float mToeAngle;
		public char8[12] structgen_pad0;
	}
	[CRepr] public struct PxVehicleSuspensionData
	{
		public float mSpringStrength;
		public float mSpringDamperRate;
		public float mMaxCompression;
		public float mMaxDroop;
		public float mSprungMass;
		public float mCamberAtRest;
		public float mCamberAtMaxCompression;
		public float mCamberAtMaxDroop;
		public char8[16] structgen_pad0;
	}
	[CRepr] public struct PxVehicleAntiRollBarData
	{
		public uint32 mWheel0;
		public uint32 mWheel1;
		public float mStiffness;
		public char8[4] structgen_pad0;
	}
	[CRepr] public struct PxVehicleTireData
	{
		public float mLatStiffX;
		public float mLatStiffY;
		public float mLongitudinalStiffnessPerUnitGravity;
		public float mCamberStiffnessPerUnitGravity;
		public float[2] mFrictionVsSlipGraph;
		public uint32 mType;
		public char8[20] structgen_pad0;
	}
	[CRepr] public struct PxVehicleWheels4SimData;
	[CRepr] public struct PxVehicleWheelsSimData
	{
		public char8[96] structgen_pad0;
	}
	[CRepr] public struct PxVehicleWheels4DynData;
	[CRepr] public struct PxVehicleTireForceCalculator;
	[CRepr] public struct PxVehicleWheelsDynData
	{
		public char8[48] structgen_pad0;
	}
	[CRepr] public struct PxVehicleWheels
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public PxVehicleWheelsSimData mWheelsSimData;
		public PxVehicleWheelsDynData mWheelsDynData;
		public PxRigidDynamic* mActor;
		public char8[5] structgen_pad2;
		public uint8 mType;
		public uint8[14] mPad0;
		public char8[4] structgen_pad3;
	}
	[CRepr] public struct PxVehicleDriveSimData
	{
		public PxVehicleEngineData mEngine;
		public PxVehicleGearsData mGears;
		public PxVehicleClutchData mClutch;
		public PxVehicleAutoBoxData mAutoBox;
	}
	[CRepr] public struct PxVehicleDriveDynData
	{
		public float[16] mControlAnalogVals;
		public bool mUseAutoGears;
		public bool mGearUpPressed;
		public bool mGearDownPressed;
		public char8[1] structgen_pad0;
		public uint32 mCurrentGear;
		public uint32 mTargetGear;
		public float mEnginespeed;
		public float mGearSwitchTime;
		public float mAutoBoxSwitchTime;
		public char8[8] structgen_pad1;
	}
	[CRepr] public struct PxVehicleDrive
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public PxVehicleWheelsSimData mWheelsSimData;
		public PxVehicleWheelsDynData mWheelsDynData;
		public PxRigidDynamic* mActor;
		public char8[5] structgen_pad2;
		public uint8 mType;
		public uint8[14] mPad0;
		public char8[4] structgen_pad3;
		public PxVehicleDriveDynData mDriveDynData;
	}
	[CRepr] public struct PxVehicleDriveSimData4W
	{
		public PxVehicleEngineData mEngine;
		public PxVehicleGearsData mGears;
		public PxVehicleClutchData mClutch;
		public PxVehicleAutoBoxData mAutoBox;
		public char8[48] structgen_pad0;
	}
	[CRepr] public struct PxVehicleDrive4W
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public PxVehicleWheelsSimData mWheelsSimData;
		public PxVehicleWheelsDynData mWheelsDynData;
		public PxRigidDynamic* mActor;
		public char8[5] structgen_pad2;
		public uint8 mType;
		public uint8[14] mPad0;
		public char8[4] structgen_pad3;
		public PxVehicleDriveDynData mDriveDynData;
		public PxVehicleDriveSimData4W mDriveSimData;
	}
	[CRepr] public struct PxVehicleDriveTank
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public PxVehicleWheelsSimData mWheelsSimData;
		public PxVehicleWheelsDynData mWheelsDynData;
		public PxRigidDynamic* mActor;
		public char8[5] structgen_pad2;
		public uint8 mType;
		public uint8[14] mPad0;
		public char8[4] structgen_pad3;
		public PxVehicleDriveDynData mDriveDynData;
		public PxVehicleDriveSimData mDriveSimData;
		public char8[16] structgen_pad4;
	}
	[CRepr] public struct PxVehicleDrivableSurfaceType
	{
		public uint32 mType;
	}
	[CRepr] public struct PxVehicleDrivableSurfaceToTireFrictionPairs
	{
		public char8[48] structgen_pad0;
	}
	[CRepr] public struct PxWheelQueryResult
	{
		public PxVec3 suspLineStart;
		public PxVec3 suspLineDir;
		public float suspLineLength;
		public bool isInAir;
		public char8[3] structgen_pad0;
		public PxActor* tireContactActor;
		public PxShape* tireContactShape;
		public PxMaterial* tireSurfaceMaterial;
		public uint32 tireSurfaceType;
		public PxVec3 tireContactPoint;
		public PxVec3 tireContactNormal;
		public float tireFriction;
		public float suspJounce;
		public float suspSpringForce;
		public PxVec3 tireLongitudinalDir;
		public PxVec3 tireLateralDir;
		public float longitudinalSlip;
		public float lateralSlip;
		public float steerAngle;
		public PxTransform localPose;
	}
	[CRepr] public struct PxVehicleWheelConcurrentUpdateData
	{
		public char8[64] structgen_pad0;
	}
	[CRepr] public struct PxVehicleConcurrentUpdateData
	{
		public PxVehicleWheelConcurrentUpdateData* concurrentWheelUpdates;
		public uint32 nbConcurrentWheelUpdates;
		public char8[28] structgen_pad0;
	}
	[CRepr] public struct PxVehicleWheelQueryResult
	{
		public PxWheelQueryResult* wheelQueryResults;
		public uint32 nbWheelQueryResults;
		public char8[4] structgen_pad0;
	}
	[CRepr] public struct PxVehicleGraph
	{
		public char8[15840] structgen_pad0;
	}
	[CRepr] public struct PxVehicleTelemetryData
	{
		public char8[48] structgen_pad0;
	}
	[CRepr] public struct PxVehicleDriveSimDataNW
	{
		public PxVehicleEngineData mEngine;
		public PxVehicleGearsData mGears;
		public PxVehicleClutchData mClutch;
		public PxVehicleAutoBoxData mAutoBox;
		public char8[16] structgen_pad0;
	}
	[CRepr] public struct PxVehicleDriveNW
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public PxVehicleWheelsSimData mWheelsSimData;
		public PxVehicleWheelsDynData mWheelsDynData;
		public PxRigidDynamic* mActor;
		public char8[5] structgen_pad2;
		public uint8 mType;
		public uint8[14] mPad0;
		public char8[4] structgen_pad3;
		public PxVehicleDriveDynData mDriveDynData;
		public PxVehicleDriveSimDataNW mDriveSimData;
	}
	[CRepr] public struct PxVehicleDrive4WRawInputData
	{
		public char8[40] structgen_pad0;
	}
	[CRepr] public struct PxVehicleKeySmoothingData
	{
		public float[16] mRiseRates;
		public float[16] mFallRates;
	}
	[CRepr] public struct PxFixedSizeLookupTable_8__Pod
	{
		public float[16] mDataPairs;
		public uint32 mNbDataPairs;
		public uint32[3] mPad;
	}
	[CRepr] public struct PxVehiclePadSmoothingData
	{
		public float[16] mRiseRates;
		public float[16] mFallRates;
	}
	[CRepr] public struct PxVehicleDriveNWRawInputData
	{
		public char8[40] structgen_pad0;
	}
	[CRepr] public struct PxVehicleDriveTankRawInputData
	{
		public char8[32] structgen_pad0;
	}
	[CRepr] public struct PxVehicleCopyDynamicsMap
	{
		public uint8[20] sourceWheelIds;
		public uint8[20] targetWheelIds;
	}
	[CRepr] public struct PxVehicleGraphChannelDesc
	{
		public float mMinY;
		public float mMaxY;
		public float mMidY;
		public PxVec3 mColorLow;
		public PxVec3 mColorHigh;
		public char8[4] structgen_pad0;
		public char8* mTitle;
	}
	[CRepr] public struct PxVehicleGraphDesc
	{
		public char8[32] structgen_pad0;
	}
	[CRepr] public struct PxVehicleNoDrive
	{
		public char8[8] structgen_pad0;
		public uint16 mConcreteType;
		public PxBaseFlags mBaseFlags;
		public char8[4] structgen_pad1;
		public PxVehicleWheelsSimData mWheelsSimData;
		public PxVehicleWheelsDynData mWheelsDynData;
		public PxRigidDynamic* mActor;
		public char8[5] structgen_pad2;
		public uint8 mType;
		public uint8[14] mPad0;
		public char8[36] structgen_pad3;
	}
	[CRepr] public struct PxProfileScoped
	{
		public PxProfilerCallback* mCallback;
		public char8* mEventName;
		public void* mProfilerData;
		public uint64 mContextId;
		public bool mDetached;
		public char8[7] structgen_pad0;
	}
	[CRepr] public struct PxPvdTransport
	{
		// public void* vtable_;
	}
	[CRepr] public struct PxPvdInstrumentationFlags
	{
		public uint8 mBits;
	}

	public enum PxGeometryType : int32
	{
		eSPHERE,
		ePLANE,
		eCAPSULE,
		eBOX,
		eCONVEXMESH,
		eTRIANGLEMESH,
		eHEIGHTFIELD,
		eGEOMETRY_COUNT,//!< internal use only!
		eINVALID = -1//!< internal use only!
	}
}
