#pragma once

#include "CoreMinimal.h"

/** Minimal 3x3 matrix — UE's FMatrix is 4x4 (affine), we need plain 3x3 for spatial-inertia blocks. */
struct FMat3
{
	float M[3][3] = {};

	static FMat3 Zero() { return FMat3(); }

	static FMat3 Identity()
	{
		FMat3 R;
		R.M[0][0] = R.M[1][1] = R.M[2][2] = 1.0f;
		return R;
	}

	static FMat3 Diagonal(const FVector& D)
	{
		FMat3 R;
		R.M[0][0] = static_cast<float>(D.X);
		R.M[1][1] = static_cast<float>(D.Y);
		R.M[2][2] = static_cast<float>(D.Z);
		return R;
	}

	/** Cross-product (skew-symmetric) matrix such that Skew(v) * x == v ^ x */
	static FMat3 Skew(const FVector& V)
	{
		FMat3 R;
		R.M[0][1] = -static_cast<float>(V.Z); R.M[0][2] =  static_cast<float>(V.Y);
		R.M[1][0] =  static_cast<float>(V.Z); R.M[1][2] = -static_cast<float>(V.X);
		R.M[2][0] = -static_cast<float>(V.Y); R.M[2][1] =  static_cast<float>(V.X);
		return R;
	}

	/**
	 * NOTE: UE's FMatrix uses row-vector convention (out = V * M, see
	 * FMatrix::TransformVector), so FRotationMatrix::Make(Q).M is the
	 * TRANSPOSE of the matrix R such that R * V (ordinary column-vector
	 * product, as implemented by FMat3::operator*) equals Q.RotateVector(V).
	 * Transpose on extraction so FMat3::FromRotation(Q) * V == Q.RotateVector(V).
	 */
	static FMat3 FromRotation(const FQuat& Q)
	{
		const FMatrix M44 = FRotationMatrix::Make(Q);
		FMat3 R;
		for (int32 r = 0; r < 3; ++r)
			for (int32 c = 0; c < 3; ++c)
				R.M[r][c] = M44.M[c][r];
		return R;
	}

	/** Outer product A * B^T. Used to build rank-1 updates (e.g. U U^T / d in the ABA reduction). */
	static FMat3 Outer(const FVector& A, const FVector& B)
	{
		FMat3 R;
		const float a[3] = { (float)A.X, (float)A.Y, (float)A.Z };
		const float b[3] = { (float)B.X, (float)B.Y, (float)B.Z };
		for (int32 r = 0; r < 3; ++r)
			for (int32 c = 0; c < 3; ++c)
				R.M[r][c] = a[r] * b[c];
		return R;
	}

	FMat3 Transpose() const
	{
		FMat3 R;
		for (int32 r = 0; r < 3; ++r)
			for (int32 c = 0; c < 3; ++c)
				R.M[r][c] = M[c][r];
		return R;
	}

	FMat3 operator+(const FMat3& O) const
	{
		FMat3 R;
		for (int32 r = 0; r < 3; ++r) for (int32 c = 0; c < 3; ++c) R.M[r][c] = M[r][c] + O.M[r][c];
		return R;
	}

	FMat3 operator-(const FMat3& O) const
	{
		FMat3 R;
		for (int32 r = 0; r < 3; ++r) for (int32 c = 0; c < 3; ++c) R.M[r][c] = M[r][c] - O.M[r][c];
		return R;
	}

	FMat3 operator*(float S) const
	{
		FMat3 R;
		for (int32 r = 0; r < 3; ++r) for (int32 c = 0; c < 3; ++c) R.M[r][c] = M[r][c] * S;
		return R;
	}

	FMat3 operator*(const FMat3& O) const
	{
		FMat3 R;
		for (int32 r = 0; r < 3; ++r)
			for (int32 c = 0; c < 3; ++c)
			{
				float S = 0.0f;
				for (int32 k = 0; k < 3; ++k) S += M[r][k] * O.M[k][c];
				R.M[r][c] = S;
			}
		return R;
	}

	FVector operator*(const FVector& V) const
	{
		return FVector(
			M[0][0] * V.X + M[0][1] * V.Y + M[0][2] * V.Z,
			M[1][0] * V.X + M[1][1] * V.Y + M[1][2] * V.Z,
			M[2][0] * V.X + M[2][1] * V.Y + M[2][2] * V.Z);
	}
};

/** Spatial motion/force vector: angular part on top, linear part on bottom (Featherstone convention). */
struct FSpatialVec
{
	FVector Ang = FVector::ZeroVector;
	FVector Lin = FVector::ZeroVector;

	FSpatialVec operator+(const FSpatialVec& O) const { return { Ang + O.Ang, Lin + O.Lin }; }
	FSpatialVec operator-(const FSpatialVec& O) const { return { Ang - O.Ang, Lin - O.Lin }; }
	FSpatialVec operator*(float S) const { return { Ang * S, Lin * S }; }

	static float Dot(const FSpatialVec& A, const FSpatialVec& B)
	{
		return static_cast<float>(FVector::DotProduct(A.Ang, B.Ang) + FVector::DotProduct(A.Lin, B.Lin));
	}
};

/** Spatial cross product for MOTION vectors: v x* m (used for the gyroscopic bias term). */
inline FSpatialVec SpatialCrossMotion(const FSpatialVec& V, const FSpatialVec& Mo)
{
	return {
		FVector::CrossProduct(V.Ang, Mo.Ang),
		FVector::CrossProduct(V.Ang, Mo.Lin) + FVector::CrossProduct(V.Lin, Mo.Ang)
	};
}

/** Spatial cross product for FORCE (dual) vectors: v x* f. */
inline FSpatialVec SpatialCrossForce(const FSpatialVec& V, const FSpatialVec& F)
{
	return {
		FVector::CrossProduct(V.Ang, F.Ang) + FVector::CrossProduct(V.Lin, F.Lin),
		FVector::CrossProduct(V.Ang, F.Lin)
	};
}

/**
 * Spatial inertia about some reference point, in block form:
 *   [ Irot   H  ]
 *   [ H^T   MBlock ]
 * Irot: rotational inertia about the reference point (NOT necessarily the CoM).
 * H: linear/angular coupling, = Mass * Skew(CoMOffsetFromReferencePoint) for a
 *    single rigid body when the reference point isn't the CoM. Zero when the
 *    reference point IS the CoM.
 * MBlock: for a single, not-yet-combined rigid body this is Mass * Identity —
 *    but after ABA combines a child's reduced inertia into its parent, MBlock
 *    generally becomes a full symmetric 3x3 matrix, NOT a scalar. That's why
 *    this is stored as FMat3 rather than a float: treating it as scalar would
 *    be silently wrong as soon as any reduction has happened up the chain.
 */
struct FSpatialInertia
{
	FMat3 Irot;
	FMat3 H;
	FMat3 MBlock;

	FSpatialInertia operator+(const FSpatialInertia& O) const
	{
		return { Irot + O.Irot, H + O.H, MBlock + O.MBlock };
	}

	/** Apply this inertia to a spatial velocity, producing a spatial momentum/force. */
	FSpatialVec Apply(const FSpatialVec& V) const
	{
		return {
			Irot * V.Ang + H * V.Lin,
			H.Transpose() * V.Ang + MBlock * V.Lin
		};
	}

	static FSpatialInertia FromRigidBody(float Mass, const FMat3& IrotAboutRef, const FMat3& HAboutRef)
	{
		FSpatialInertia R;
		R.Irot = IrotAboutRef;
		R.H = HAboutRef;
		R.MBlock = FMat3::Identity() * Mass;
		return R;
	}

	/**
	 * Translate this inertia (currently about point B) to be about point A,
	 * where R = worldPos(B) - worldPos(A). Rotation-free (world-aligned axes
	 * throughout — see CreatureABASolver.h for why that's valid here).
	 * Derived from the Plucker translation transform I_A = X^T I_B X with
	 * X = [[Id,0],[-Skew(R),Id]]; the general (non-isotropic MBlock) form is
	 * what makes this correct for composite/reduced inertias, not just leaves.
	 */
	FSpatialInertia TranslatedTo(const FVector& R) const
	{
		const FMat3 Rx = FMat3::Skew(R);
		FSpatialInertia Out;
		Out.Irot = Irot - (H * Rx) + (Rx * H.Transpose()) - (Rx * MBlock * Rx);
		Out.H = H + Rx * MBlock;
		Out.MBlock = MBlock;
		return Out;
	}
};

/** Translate a spatial FORCE vector (moment;force) from point B to point A, R = posB - posA. */
inline FSpatialVec TranslateForce(const FSpatialVec& F, const FVector& R)
{
	return { F.Ang + FVector::CrossProduct(R, F.Lin), F.Lin };
}

/**
 * Translate a spatial MOTION vector (velocity or acceleration) from point A to
 * point B, R = posB - posA. NOTE: for acceleration this omits the velocity-
 * product ("Coriolis") correction term a fully time-varying transform would
 * include — see CreatureABASolver.h Pass 3 comment.
 */
inline FSpatialVec TranslateMotion(const FSpatialVec& M, const FVector& R)
{
	return { M.Ang, M.Lin - FVector::CrossProduct(R, M.Ang) };
}

/** Solve A * x = b for a 6x6 spatial inertia A (Gaussian elimination, partial pivoting). */
inline FSpatialVec SolveSpatial6(const FSpatialInertia& A, const FSpatialVec& B)
{
	float Aug[6][7];
	const FMat3 Ht = A.H.Transpose();
	for (int32 r = 0; r < 3; ++r)
	{
		for (int32 c = 0; c < 3; ++c) { Aug[r][c] = A.Irot.M[r][c]; Aug[r][3 + c] = A.H.M[r][c]; }
		for (int32 c = 0; c < 3; ++c) { Aug[3 + r][c] = Ht.M[r][c]; Aug[3 + r][3 + c] = A.MBlock.M[r][c]; }
	}
	const float BArr[6] = { (float)B.Ang.X, (float)B.Ang.Y, (float)B.Ang.Z, (float)B.Lin.X, (float)B.Lin.Y, (float)B.Lin.Z };
	for (int32 r = 0; r < 6; ++r) Aug[r][6] = BArr[r];

	for (int32 Col = 0; Col < 6; ++Col)
	{
		int32 Pivot = Col;
		float Best = FMath::Abs(Aug[Col][Col]);
		for (int32 r = Col + 1; r < 6; ++r)
		{
			if (FMath::Abs(Aug[r][Col]) > Best) { Best = FMath::Abs(Aug[r][Col]); Pivot = r; }
		}
		if (Pivot != Col)
		{
			for (int32 c = 0; c < 7; ++c) { const float Tmp = Aug[Col][c]; Aug[Col][c] = Aug[Pivot][c]; Aug[Pivot][c] = Tmp; }
		}
		const float PivotVal = Aug[Col][Col];
		if (FMath::Abs(PivotVal) < KINDA_SMALL_NUMBER) continue; // near-singular; shouldn't occur for a physical inertia
		for (int32 r = 0; r < 6; ++r)
		{
			if (r == Col) continue;
			const float Factor = Aug[r][Col] / PivotVal;
			for (int32 c = Col; c < 7; ++c) Aug[r][c] -= Factor * Aug[Col][c];
		}
	}

	float Sol[6];
	for (int32 r = 0; r < 6; ++r) Sol[r] = FMath::IsNearlyZero(Aug[r][r]) ? 0.0f : Aug[r][6] / Aug[r][r];
	return { FVector(Sol[0], Sol[1], Sol[2]), FVector(Sol[3], Sol[4], Sol[5]) };
}