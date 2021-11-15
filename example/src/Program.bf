using System;
using PhysX;

namespace example
{
	class Program
	{
		// NOTE this is the snippethelloworld from physx repository, and it's a demo of direct C API, use with caution!

		static PxFoundation* foundation;
		static PxDefaultCpuDispatcher* dispatcher;
		static PxPhysics* physics;
		static PxScene* scene;
		static PxMaterial* material;
		static PxRigidStatic* groundPlane;
		static PxRigidDynamic* dynamicBody;

		static void createStack(PxTransform t, int size, float halfExtent)
		{
			PxBoxGeometry boxGeometry = PhysXAPI.BoxGeometry_new_1(halfExtent, halfExtent, halfExtent);
			PxShape* shape = PhysXAPI.Physics_createShape_mut(physics, (PxGeometry*)&boxGeometry, material);
			for (int i = 0; i < size; i++)
			{
				for (int j = 0; j < size - i; j++)
				{
					PxVec3 position = PxVec3() { x = ((j * 2) - (size - i)) * boxGeometry.halfExtents.x, y = (i * 2 + 1) * boxGeometry.halfExtents.y, z = 0 };
					PxTransform localTm = PhysXAPI.Transform_new_1(&position);
					var newTransform = PhysXAPI.Transform_transform_1(&t, &localTm);
					PxRigidDynamic* body = PhysXAPI.Physics_createRigidDynamic_mut(physics, &newTransform);
					PhysXAPI.RigidActor_attachShape_mut((PxRigidActor*)body, shape);
					PhysXAPI.RigidBodyExt_updateMassAndInertia_mut_1((PxRigidBody*)body, 10.0f, null, false);
					PhysXAPI.Scene_addActor_mut(scene, (PxActor*)body, null);
				}
			}
			PhysXAPI.Shape_release_mut(shape);
		}

		static PxRigidDynamic* createDynamic(PxTransform t, PxSphereGeometry geometry, PxVec3 velocity)
		{
			var offset = PhysXAPI.Transform_new_2(0);
			PxRigidDynamic* dynamic = PhysXAPI.CreateDynamic(physics, &t, (PxGeometry*)&geometry, material, 10.0f, &offset);
			PhysXAPI.RigidBody_setAngularDamping_mut((PxRigidBody*)dynamic, 0.5f);
			PhysXAPI.RigidBody_setLinearVelocity_mut((PxRigidBody*)dynamic, &velocity, true);
			PhysXAPI.Scene_addActor_mut(scene, (PxActor*)dynamic, null);
			return dynamic;
		}

		static void initPhysics(bool interactive)
		{
			foundation = PhysXAPI.create_foundation();
			physics = PhysXAPI.create_physics(foundation);

			dispatcher = PhysXAPI.DefaultCpuDispatcherCreate(2, null);

			PhysX.PxSceneDesc sceneDesc = PhysX.PxSceneDesc();
			sceneDesc.gravity = PxVec3() { x = 0.0f, y = -9.81f, z = 0.0f };
			sceneDesc.cpuDispatcher = (PxCpuDispatcher*)dispatcher;
			sceneDesc.filterShader = PhysXAPI.get_default_simulation_filter_shader();
			sceneDesc.tolerancesScale = *PhysXAPI.Physics_getTolerancesScale(physics);
			scene = PhysXAPI.Physics_createScene_mut(physics, &sceneDesc);
			material = PhysXAPI.Physics_createMaterial_mut(physics, 0.5f, 0.5f, 0.6f);
			var plane = PxPlane() { n = PxVec3() { x = 0, y = 1, z = 0 }, d = 0 };

			groundPlane = PhysXAPI.CreatePlane(physics, &plane, material);
			PhysXAPI.Scene_addActor_mut(scene, (PxActor*)groundPlane, null);
			var stackZ = 10.0f;
			for (int i = 0; i < 5; i++)
			{
				var position = PxVec3() { x = 0, y = 0, z = stackZ -= 10.0f };
				createStack(PhysXAPI.Transform_new_1(&position), 10, 2.0f);
			}

			var position = PxVec3() { x = 0, y = 40, z = 100 };
			var sphere = PhysXAPI.SphereGeometry_new_1(10);
			dynamicBody = createDynamic(PhysXAPI.Transform_new_1(&position), sphere, PxVec3() { x = 0, y = -50, z = -100 });
		}

		static void stepPhysics(bool interactive)
		{
			PhysXAPI.Scene_simulate_mut(scene, 1.0f / 60.0f, null, null, 0, true);
			PhysXAPI.Scene_fetchResults_mut(scene, true, null);
		}

		static void cleanupPhysics(bool interactive)
		{
			PhysXAPI.Scene_release_mut(scene);
			PhysXAPI.DefaultCpuDispatcher_release_mut(dispatcher);
			PhysXAPI.Physics_release_mut(physics);
			PhysXAPI.Foundation_release_mut(foundation);
		}

		public static int Main()
		{
			const int frameCount = 100;
			initPhysics(false);
			for (int i = 0; i < frameCount; i++)
				stepPhysics(false);

			var globalPose = PhysXAPI.RigidActor_getGlobalPose((PxRigidActor*)dynamicBody);

			var targetPosition = PxVec3() { x = 0.672708f, y = 42.051411f, z = -19.838844f };

			var xDifference = Math.Abs(globalPose.p.x - targetPosition.x);
			if (xDifference > 0.0001f)
			{
				Console.WriteLine(scope $"X failed by {xDifference}!");
			}
			var yDifference = Math.Abs(globalPose.p.y - targetPosition.y);
			if (yDifference > 0.0001f)
			{
				Console.WriteLine(scope $"Y failed by {yDifference}!");
			}
			var zDifference = Math.Abs(globalPose.p.z - targetPosition.z);
			if (zDifference > 0.0001f)
			{
				Console.WriteLine(scope $"Z failed by {zDifference}!");
			}
			Console.WriteLine(scope $"{globalPose.p.x}, {globalPose.p.y}, {globalPose.p.z}");
			cleanupPhysics(false);
			return 0;
		}
	}
}
