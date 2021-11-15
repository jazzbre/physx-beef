using System;
using PhysX;

namespace example
{
	class Program
	{
		public static int Main()
		{
			var foundation = PhysXAPI.create_foundation();
			var physics = PhysXAPI.create_physics(foundation);
			PhysX.PxSceneDesc sceneDesc = default;
			var scene = PhysXAPI.Physics_createScene_mut(physics, &sceneDesc);
			return 0;
		}
	}
}
