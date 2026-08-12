#pragma once

// Ground contact for AgentSolver's ABA creature. No new solver math: contact
// is a penalty force (spring-damper + clamped Coulomb friction) applied
// through FCreatureBatchState::ApplyForceAtPoint, which already does exactly
// "accumulate an external wrench at a world point on a body" — the solver's
// bias pass folds it in every step like any other external force. This
// matches the project's existing "self-consistent dynamics over exact
// physical accuracy" stance (see CreatureBatchSolver.h's Coriolis-omission
// note) rather than building a full LCP/constraint contact solver.
//
// Ground is an infinite flat plane (GroundZ + GroundNormal in FContactParams)
// — a heightfield/uneven terrain is a possible future extension, not needed
// yet.

#include "CoreMinimal.h"
#include "CreatureBatchState.h"

#if WITH_EDITOR
// Only needs the mass-profile asset type now — BuildMutoContactPoints reads
// CanTouchGround/LimbIndex straight from the already-built FCreatureTopology,
// no MutoTopology::-namespace helpers required anymore.
#include "UMassMuscleProfileAsset.h"
#endif

namespace CreatureGroundContact
{
	/** One contact point: a fixed offset (in BodyIndex's own rest frame) tracked every step. */
	struct FContactPointDef
	{
		int32 BodyIndex = INDEX_NONE;
		FVector LocalOffset = FVector::ZeroVector; // sphere CENTER, in BodyIndex's own rest frame — see Radius
		FName DebugName;
		int32 LimbIndex = INDEX_NONE; // which physical limb this belongs to (0..7 for Muto) — for future reward code to group by limb

		// Collision sphere radius at LocalOffset (see FMassMuscleDataMass::
		// Radius) — the actual ground-contact surface is this far below the
		// sphere's center along the ground normal, not the center itself.
		// Zero reproduces the old "point contact at LocalOffset exactly"
		// behavior (e.g. every synthetic test's hand-built points).
		float Radius = 0.0f;

		// Capsule half-height (see FMassMuscleDataMass::CapsuleHalfHeight):
		// LocalOffset is the capsule's fixed END cap (the tip); the START cap
		// sits CapsuleHalfHeight*2 back toward the body's own origin, along
		// the BodyIndex->LocalOffset axis. Zero (default) collapses the
		// capsule to a single sphere at LocalOffset — the original behavior,
		// and what every synthetic test's hand-built point still gets.
		float CapsuleHalfHeight = 0.0f;
	};

	struct FContactParams
	{
		float GroundZ = 0.0f;
		FVector GroundNormal = FVector::UpVector;

		// Defaults tuned against CreatureGroundContactTest.cpp's drop test —
		// scale these with body mass for other setups (see that test's
		// comments for the reasoning: SpringK ~ Mass*Gravity/DesiredSag).
		float SpringK = 5000.0f;
		float DamperK = 200.0f;
		float FrictionK = 50.0f;
		float FrictionCoefficient = 0.8f;
	};

	/** Per-point-per-env output, queryable by a future reward system. Not consumed anywhere yet — see project roadmap/memory. */
	struct FContactPointState
	{
		float NormalForce = 0.0f;
		bool bTouching = false;
	};

	/**
	 * Computes and APPLIES penalty-based contact forces for every point x every
	 * env via Batch.ApplyForceAtPoint. Call once per step, BEFORE Solver.Step()
	 * (external forces are picked up by Step()'s bias pass).
	 *
	 * IMPORTANT: ApplyForceAtPoint accumulates into ExtForceX/Y/Z and Step()
	 * never clears it (by design — see FCreatureBatchState's comment, meant
	 * for one-shot vs. persistent hits). Contact forces are recomputed fresh
	 * every step from current penetration/velocity, so the caller MUST
	 * Batch.ClearExternalForces(Env) (every env) before calling this each
	 * step, or forces accumulate forever:
	 *
	 *   for (int32 Env = 0; Env < Batch.GetNumEnvs(); ++Env) { Batch.ClearExternalForces(Env); }
	 *   CreatureGroundContact::ApplyGroundContactForces(Batch, Topo, Points, Params);
	 *   Solver.Step(Batch, Dt);
	 */
	inline void ApplyGroundContactForces(
		FCreatureBatchState& Batch,
		const FCreatureTopology& Topo,
		const TArray<FContactPointDef>& ContactPoints,
		const FContactParams& Params,
		TArray<FContactPointState>* OutState = nullptr)
	{
		const int32 NumEnvs = Batch.GetNumEnvs();
		if (OutState)
		{
			OutState->SetNumZeroed(ContactPoints.Num() * NumEnvs);
		}

		for (int32 PointIdx = 0; PointIdx < ContactPoints.Num(); ++PointIdx)
		{
			const FContactPointDef& Point = ContactPoints[PointIdx];

			// A capsule is approximated as two sphere contacts, one at each
			// end cap: LocalOffset (the tip, fixed) and LocalOffset pulled
			// back toward the body's own origin by CapsuleHalfHeight*2 (see
			// FContactPointDef::CapsuleHalfHeight). When CapsuleHalfHeight==0
			// both ends coincide, which would double-apply the sphere
			// contact at the same point — skip the start cap in that case so
			// behavior (and force magnitude) is unchanged from the old
			// single-sphere model, matching every synthetic test's hand-built
			// points (CapsuleHalfHeight defaults to 0).
			FVector LocalEnds[2];
			int32 NumEnds = 1;
			LocalEnds[0] = Point.LocalOffset;
			if (Point.CapsuleHalfHeight > 0.0f)
			{
				const FVector Axis = Point.LocalOffset.GetSafeNormal();
				LocalEnds[1] = Point.LocalOffset - Axis * (Point.CapsuleHalfHeight * 2.0f);
				NumEnds = 2;
			}

			for (int32 Env = 0; Env < NumEnvs; ++Env)
			{
				const FQuat Rot = Batch.GetBodyRot(Point.BodyIndex, Env);
				const FVector BodyPos = Batch.GetBodyPos(Point.BodyIndex, Env);
				const int32 Idx = Batch.BodyIndex(Point.BodyIndex, Env);
				const FVector AngVel(Batch.AngVelX[Idx], Batch.AngVelY[Idx], Batch.AngVelZ[Idx]);
				const FVector LinVel(Batch.LinVelX[Idx], Batch.LinVelY[Idx], Batch.LinVelZ[Idx]);

				FContactPointState State;
				for (int32 EndIdx = 0; EndIdx < NumEnds; ++EndIdx)
				{
					const FVector SphereCenter = BodyPos + Rot.RotateVector(LocalEnds[EndIdx]);
					// The actual contact SURFACE is Radius below the sphere's
					// center along the ground normal — using the center itself
					// (Radius==0, e.g. every synthetic test's hand-built points)
					// reproduces the old exact-point-contact behavior.
					const FVector WorldPoint = SphereCenter - Point.Radius * Params.GroundNormal;

					const FVector PointVel = LinVel + FVector::CrossProduct(AngVel, WorldPoint - BodyPos);

					const float Penetration = Params.GroundZ - static_cast<float>(WorldPoint.Z);
					if (Penetration <= 0.0f)
					{
						continue;
					}

					const float VelAlongNormal = static_cast<float>(FVector::DotProduct(PointVel, Params.GroundNormal));
					// Spring pushes out proportional to penetration; damper
					// only resists closing velocity (a contact can push, not
					// pull, hence the outer Max(0,...) too).
					const float NormalForceMag = FMath::Max(0.0f, Params.SpringK * Penetration - Params.DamperK * VelAlongNormal);
					const FVector NormalForceVec = NormalForceMag * Params.GroundNormal;

					const FVector TangentialVel = PointVel - VelAlongNormal * Params.GroundNormal;
					const float MaxFriction = Params.FrictionCoefficient * NormalForceMag;
					FVector FrictionForceVec = -Params.FrictionK * TangentialVel;
					if (FrictionForceVec.SizeSquared() > MaxFriction * MaxFriction)
					{
						FrictionForceVec = FrictionForceVec.GetSafeNormal() * MaxFriction; // clamp to the Coulomb friction cone
					}

					Batch.ApplyForceAtPoint(Point.BodyIndex, Env, NormalForceVec + FrictionForceVec, WorldPoint);

					State.NormalForce += NormalForceMag;
					State.bTouching = true;
				}

				if (OutState)
				{
					(*OutState)[PointIdx * NumEnvs + Env] = State;
				}
			}
		}
	}

#if WITH_EDITOR
	/**
	 * Derives ground-contact points for Muto straight from the authored
	 * MassProfile_Muto CanTouchGround flag, one point (at the body's own
	 * origin) per body whose bone is flagged — no structural guess about
	 * "the last N bones of the chain are the foot". This matters because the
	 * real data doesn't follow a uniform pattern: legs are touchable on
	 * {Feet, FeetTip} but NOT the shin-equivalent Knee2, while arms are
	 * touchable on {Elbow3, Hand} but NOT the Tip past the hand — a fixed
	 * "last 3 bones" rule (the previous approach here) gets legs wrong.
	 * M limbs come out excluded automatically since every M bone has
	 * CanTouchGround=false in the authored data (matches the established
	 * fact that M limbs never touch the ground).
	 *
	 * Requires BodyDebugNames (MutoTopology::BuildMutoTopology's optional
	 * OutBodyDebugNames out-param) to map body index -> bone name for the
	 * MassAsset lookup. Per-point LimbIndex comes straight from
	 * Topo.BodyLimbIndex — one slot per archetype x side (0..7), matching
	 * NumLimbs = Archetypes.Num() * 2 (see MutoTopology.h's LimbIndex
	 * assignment; this used to have to re-derive its own parallel counter
	 * here because that assignment only distinguished archetype, not L/R
	 * side, but that's fixed now).
	 */
	inline TArray<FContactPointDef> BuildMutoContactPoints(const FCreatureTopology& Topo, UMassMuscleProfileAssetMass* MassAsset, const TArray<FName>& BodyDebugNames)
	{
		TArray<FContactPointDef> Points;
		if (!MassAsset)
		{
			return Points;
		}

		for (int32 Body = 1; Body < Topo.NumBodies; ++Body)
		{
			if (!BodyDebugNames.IsValidIndex(Body))
			{
				continue;
			}
			const int32 MassIdx = MassAsset->FindBoneByName(BodyDebugNames[Body]);
			if (MassIdx != INDEX_NONE && MassAsset->Mass[MassIdx].CanTouchGround)
			{
				// LocalOffset = the fused Tip's rest offset if this body has
				// one (ZeroVector otherwise) — Feet/Hand's own joint origin
				// isn't the visual extent of the foot/hand, the fused,
				// unarticulated Tip bone hanging past it is (see
				// FCreatureTopology::BodyFusedTipOffset's comment). Radius =
				// the authored per-bone collision radius (see
				// FMassMuscleDataMass::Radius) — REPLACES the old computed
				// "ground-alignment correction" heuristic entirely (see
				// project memory/roadmap for that saga): rather than
				// forcing every point to reach an artificially-equalized
				// height, each point now uses real, author-provided
				// geometry (LocalOffset reaches to the bone's visual end,
				// Radius accounts for its thickness) — if that data is
				// authored accurately, every touching limb naturally lands
				// at the same height because that's what the real mesh
				// does, not because the code forced it to. CapsuleHalfHeight
				// (see FMassMuscleDataMass::CapsuleHalfHeight) extends this
				// into a real capsule back up the limb instead of a single
				// point at the tip.
				Points.Add({ Body, Topo.BodyFusedTipOffset[Body], BodyDebugNames[Body], Topo.BodyLimbIndex[Body], Topo.BodyRadius[Body], Topo.BodyCapsuleHalfHeight[Body] });
			}
		}

		return Points;
	}
#endif // WITH_EDITOR
}
