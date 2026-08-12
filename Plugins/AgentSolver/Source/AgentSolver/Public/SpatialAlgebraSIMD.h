#pragma once

#include "CoreMinimal.h"
#include "Math/VectorRegister.h"
#include "SpatialAlgebra.h"
#include <immintrin.h>

/**
 * 8-wide (AVX2) mirror of SpatialAlgebra.h, batched across environments —
 * one lane per env, matching FCreatureBatchState::SIMDWidth. Every type
 * here holds one component per env in a single __m256, so an FVec3x8 is
 * 8 envs' worth of one body's position (say), not one env's xyz spread
 * across lanes. This is what lets CreatureBatchSolver's SIMD passes load
 * straight from FCreatureBatchState's body-major/env-minor float arrays
 * with no gather/shuffle.
 *
 * Loads/stores are unaligned (_mm256_loadu/storeu) since TArray's
 * allocator isn't guaranteed 32-byte aligned.
 */

FORCEINLINE __m256 LoadUint8AsFloat8(const uint8* Ptr)
{
	const __m128i Bytes = _mm_loadl_epi64(reinterpret_cast<const __m128i*>(Ptr));
	const __m256i Ints = _mm256_cvtepu8_epi32(Bytes);
	return _mm256_cvtepi32_ps(Ints);
}

/** sin/cos of 8 angles at once, via two calls into the engine's already-validated 4-wide VectorSinCos. */
FORCEINLINE void VectorSinCos8(__m256 Angles, __m256& OutSin, __m256& OutCos)
{
	VectorRegister4Float Lo = _mm256_extractf128_ps(Angles, 0);
	VectorRegister4Float Hi = _mm256_extractf128_ps(Angles, 1);
	VectorRegister4Float SinLo, CosLo, SinHi, CosHi;
	VectorSinCos(&SinLo, &CosLo, &Lo);
	VectorSinCos(&SinHi, &CosHi, &Hi);
	OutSin = _mm256_insertf128_ps(_mm256_castps128_ps256(SinLo), SinHi, 1);
	OutCos = _mm256_insertf128_ps(_mm256_castps128_ps256(CosLo), CosHi, 1);
}

struct FVec3x8
{
	__m256 X = _mm256_setzero_ps();
	__m256 Y = _mm256_setzero_ps();
	__m256 Z = _mm256_setzero_ps();

	static FVec3x8 Zero() { return FVec3x8(); }

	static FVec3x8 Load(const float* Xp, const float* Yp, const float* Zp)
	{
		FVec3x8 R;
		R.X = _mm256_loadu_ps(Xp);
		R.Y = _mm256_loadu_ps(Yp);
		R.Z = _mm256_loadu_ps(Zp);
		return R;
	}

	void Store(float* Xp, float* Yp, float* Zp) const
	{
		_mm256_storeu_ps(Xp, X);
		_mm256_storeu_ps(Yp, Y);
		_mm256_storeu_ps(Zp, Z);
	}

	/** Same value in every lane — for per-body constants shared across all envs (topology data). */
	static FVec3x8 Broadcast(const FVector& V)
	{
		FVec3x8 R;
		R.X = _mm256_set1_ps(static_cast<float>(V.X));
		R.Y = _mm256_set1_ps(static_cast<float>(V.Y));
		R.Z = _mm256_set1_ps(static_cast<float>(V.Z));
		return R;
	}

	FVec3x8 operator+(const FVec3x8& O) const { return { _mm256_add_ps(X, O.X), _mm256_add_ps(Y, O.Y), _mm256_add_ps(Z, O.Z) }; }
	FVec3x8 operator-(const FVec3x8& O) const { return { _mm256_sub_ps(X, O.X), _mm256_sub_ps(Y, O.Y), _mm256_sub_ps(Z, O.Z) }; }
	FVec3x8 operator*(__m256 S) const { return { _mm256_mul_ps(X, S), _mm256_mul_ps(Y, S), _mm256_mul_ps(Z, S) }; }

	static __m256 Dot(const FVec3x8& A, const FVec3x8& B)
	{
		return _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(A.X, B.X), _mm256_mul_ps(A.Y, B.Y)), _mm256_mul_ps(A.Z, B.Z));
	}

	static FVec3x8 Cross(const FVec3x8& A, const FVec3x8& B)
	{
		return {
			_mm256_sub_ps(_mm256_mul_ps(A.Y, B.Z), _mm256_mul_ps(A.Z, B.Y)),
			_mm256_sub_ps(_mm256_mul_ps(A.Z, B.X), _mm256_mul_ps(A.X, B.Z)),
			_mm256_sub_ps(_mm256_mul_ps(A.X, B.Y), _mm256_mul_ps(A.Y, B.X))
		};
	}
};

struct FMat3x8
{
	__m256 M[3][3];

	FMat3x8()
	{
		for (int32 r = 0; r < 3; ++r)
			for (int32 c = 0; c < 3; ++c)
				M[r][c] = _mm256_setzero_ps();
	}

	static FMat3x8 Identity()
	{
		FMat3x8 R;
		R.M[0][0] = R.M[1][1] = R.M[2][2] = _mm256_set1_ps(1.0f);
		return R;
	}

	static FMat3x8 Diagonal(const FVec3x8& D)
	{
		FMat3x8 R;
		R.M[0][0] = D.X;
		R.M[1][1] = D.Y;
		R.M[2][2] = D.Z;
		return R;
	}

	/** Cross-product (skew-symmetric) matrix such that Skew(v) * x == v ^ x, matching FMat3::Skew. */
	static FMat3x8 Skew(const FVec3x8& V)
	{
		const __m256 SignMask = _mm256_set1_ps(-0.0f);
		const __m256 NegX = _mm256_xor_ps(V.X, SignMask);
		const __m256 NegY = _mm256_xor_ps(V.Y, SignMask);
		const __m256 NegZ = _mm256_xor_ps(V.Z, SignMask);

		FMat3x8 R;
		R.M[0][1] = NegZ; R.M[0][2] = V.Y;
		R.M[1][0] = V.Z;  R.M[1][2] = NegX;
		R.M[2][0] = NegY; R.M[2][1] = V.X;
		return R;
	}

	/** Outer product A * B^T, matching FMat3::Outer. */
	static FMat3x8 Outer(const FVec3x8& A, const FVec3x8& B)
	{
		FMat3x8 R;
		R.M[0][0] = _mm256_mul_ps(A.X, B.X); R.M[0][1] = _mm256_mul_ps(A.X, B.Y); R.M[0][2] = _mm256_mul_ps(A.X, B.Z);
		R.M[1][0] = _mm256_mul_ps(A.Y, B.X); R.M[1][1] = _mm256_mul_ps(A.Y, B.Y); R.M[1][2] = _mm256_mul_ps(A.Y, B.Z);
		R.M[2][0] = _mm256_mul_ps(A.Z, B.X); R.M[2][1] = _mm256_mul_ps(A.Z, B.Y); R.M[2][2] = _mm256_mul_ps(A.Z, B.Z);
		return R;
	}

	/**
	 * Quaternion -> rotation matrix such that FromRotation(Q) * V == Q.RotateVector(V),
	 * matching FMat3::FromRotation's convention (verified against it by the SIMD-vs-
	 * scalar automation test rather than hand-derived byte-parity with FRotationMatrix).
	 */
	static FMat3x8 FromRotation(const struct FQuatx8& Q);

	FMat3x8 Transpose() const
	{
		FMat3x8 R;
		for (int32 r = 0; r < 3; ++r)
			for (int32 c = 0; c < 3; ++c)
				R.M[r][c] = M[c][r];
		return R;
	}

	FMat3x8 operator+(const FMat3x8& O) const
	{
		FMat3x8 R;
		for (int32 r = 0; r < 3; ++r) for (int32 c = 0; c < 3; ++c) R.M[r][c] = _mm256_add_ps(M[r][c], O.M[r][c]);
		return R;
	}

	FMat3x8 operator-(const FMat3x8& O) const
	{
		FMat3x8 R;
		for (int32 r = 0; r < 3; ++r) for (int32 c = 0; c < 3; ++c) R.M[r][c] = _mm256_sub_ps(M[r][c], O.M[r][c]);
		return R;
	}

	FMat3x8 operator*(__m256 S) const
	{
		FMat3x8 R;
		for (int32 r = 0; r < 3; ++r) for (int32 c = 0; c < 3; ++c) R.M[r][c] = _mm256_mul_ps(M[r][c], S);
		return R;
	}

	FMat3x8 operator*(const FMat3x8& O) const
	{
		FMat3x8 R;
		for (int32 r = 0; r < 3; ++r)
		{
			for (int32 c = 0; c < 3; ++c)
			{
				__m256 S = _mm256_setzero_ps();
				for (int32 k = 0; k < 3; ++k) S = _mm256_add_ps(S, _mm256_mul_ps(M[r][k], O.M[k][c]));
				R.M[r][c] = S;
			}
		}
		return R;
	}

	FVec3x8 operator*(const FVec3x8& V) const
	{
		return {
			_mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(M[0][0], V.X), _mm256_mul_ps(M[0][1], V.Y)), _mm256_mul_ps(M[0][2], V.Z)),
			_mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(M[1][0], V.X), _mm256_mul_ps(M[1][1], V.Y)), _mm256_mul_ps(M[1][2], V.Z)),
			_mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(M[2][0], V.X), _mm256_mul_ps(M[2][1], V.Y)), _mm256_mul_ps(M[2][2], V.Z))
		};
	}
};

struct FQuatx8
{
	__m256 X = _mm256_setzero_ps();
	__m256 Y = _mm256_setzero_ps();
	__m256 Z = _mm256_setzero_ps();
	__m256 W = _mm256_set1_ps(1.0f);

	static FQuatx8 Load(const float* Xp, const float* Yp, const float* Zp, const float* Wp)
	{
		FQuatx8 R;
		R.X = _mm256_loadu_ps(Xp);
		R.Y = _mm256_loadu_ps(Yp);
		R.Z = _mm256_loadu_ps(Zp);
		R.W = _mm256_loadu_ps(Wp);
		return R;
	}

	static FQuatx8 Broadcast(const FQuat& Q)
	{
		FQuatx8 R;
		R.X = _mm256_set1_ps(static_cast<float>(Q.X));
		R.Y = _mm256_set1_ps(static_cast<float>(Q.Y));
		R.Z = _mm256_set1_ps(static_cast<float>(Q.Z));
		R.W = _mm256_set1_ps(static_cast<float>(Q.W));
		return R;
	}

	void Store(float* Xp, float* Yp, float* Zp, float* Wp) const
	{
		_mm256_storeu_ps(Xp, X);
		_mm256_storeu_ps(Yp, Y);
		_mm256_storeu_ps(Zp, Z);
		_mm256_storeu_ps(Wp, W);
	}

	/** Matches FQuat(Axis, Angle); Axis must be unit length. */
	static FQuatx8 FromAxisAngle(const FVec3x8& Axis, __m256 Angle)
	{
		const __m256 Half = _mm256_mul_ps(Angle, _mm256_set1_ps(0.5f));
		__m256 S, C;
		VectorSinCos8(Half, S, C);

		FQuatx8 R;
		R.X = _mm256_mul_ps(S, Axis.X);
		R.Y = _mm256_mul_ps(S, Axis.Y);
		R.Z = _mm256_mul_ps(S, Axis.Z);
		R.W = C;
		return R;
	}

	/** Hamilton product; A*B first applies B then A, matching FQuat::operator*. */
	friend FQuatx8 operator*(const FQuatx8& A, const FQuatx8& B)
	{
		FQuatx8 R;
		R.W = _mm256_sub_ps(_mm256_sub_ps(_mm256_sub_ps(_mm256_mul_ps(A.W, B.W), _mm256_mul_ps(A.X, B.X)), _mm256_mul_ps(A.Y, B.Y)), _mm256_mul_ps(A.Z, B.Z));
		R.X = _mm256_add_ps(_mm256_sub_ps(_mm256_add_ps(_mm256_mul_ps(A.W, B.X), _mm256_mul_ps(A.X, B.W)), _mm256_mul_ps(A.Z, B.Y)), _mm256_mul_ps(A.Y, B.Z));
		R.Y = _mm256_add_ps(_mm256_add_ps(_mm256_sub_ps(_mm256_mul_ps(A.W, B.Y), _mm256_mul_ps(A.X, B.Z)), _mm256_mul_ps(A.Y, B.W)), _mm256_mul_ps(A.Z, B.X));
		R.Z = _mm256_add_ps(_mm256_sub_ps(_mm256_add_ps(_mm256_mul_ps(A.W, B.Z), _mm256_mul_ps(A.X, B.Y)), _mm256_mul_ps(A.Y, B.X)), _mm256_mul_ps(A.Z, B.W));
		return R;
	}

	FQuatx8 GetNormalized() const
	{
		const __m256 LenSq = _mm256_add_ps(_mm256_add_ps(_mm256_mul_ps(X, X), _mm256_mul_ps(Y, Y)), _mm256_add_ps(_mm256_mul_ps(Z, Z), _mm256_mul_ps(W, W)));
		const __m256 InvLen = _mm256_div_ps(_mm256_set1_ps(1.0f), _mm256_sqrt_ps(LenSq));
		FQuatx8 R;
		R.X = _mm256_mul_ps(X, InvLen);
		R.Y = _mm256_mul_ps(Y, InvLen);
		R.Z = _mm256_mul_ps(Z, InvLen);
		R.W = _mm256_mul_ps(W, InvLen);
		return R;
	}

	static FVec3x8 RotateVector(const FQuatx8& Q, const FVec3x8& V)
	{
		const FVec3x8 QV{ Q.X, Q.Y, Q.Z };
		const FVec3x8 T = FVec3x8::Cross(QV, V) * _mm256_set1_ps(2.0f);
		return V + T * Q.W + FVec3x8::Cross(QV, T);
	}
};

FORCEINLINE FMat3x8 FMat3x8::FromRotation(const FQuatx8& Q)
{
	const __m256 x = Q.X, y = Q.Y, z = Q.Z, w = Q.W;
	const __m256 x2 = _mm256_add_ps(x, x), y2 = _mm256_add_ps(y, y), z2 = _mm256_add_ps(z, z);
	const __m256 xx = _mm256_mul_ps(x, x2), yy = _mm256_mul_ps(y, y2), zz = _mm256_mul_ps(z, z2);
	const __m256 xy = _mm256_mul_ps(x, y2), xz = _mm256_mul_ps(x, z2), yz = _mm256_mul_ps(y, z2);
	const __m256 wx = _mm256_mul_ps(w, x2), wy = _mm256_mul_ps(w, y2), wz = _mm256_mul_ps(w, z2);
	const __m256 One = _mm256_set1_ps(1.0f);

	FMat3x8 R;
	R.M[0][0] = _mm256_sub_ps(One, _mm256_add_ps(yy, zz));
	R.M[0][1] = _mm256_sub_ps(xy, wz);
	R.M[0][2] = _mm256_add_ps(xz, wy);
	R.M[1][0] = _mm256_add_ps(xy, wz);
	R.M[1][1] = _mm256_sub_ps(One, _mm256_add_ps(xx, zz));
	R.M[1][2] = _mm256_sub_ps(yz, wx);
	R.M[2][0] = _mm256_sub_ps(xz, wy);
	R.M[2][1] = _mm256_add_ps(yz, wx);
	R.M[2][2] = _mm256_sub_ps(One, _mm256_add_ps(xx, yy));
	return R;
}

struct FSpatialVecx8
{
	FVec3x8 Ang;
	FVec3x8 Lin;

	FSpatialVecx8 operator+(const FSpatialVecx8& O) const { return { Ang + O.Ang, Lin + O.Lin }; }
	FSpatialVecx8 operator-(const FSpatialVecx8& O) const { return { Ang - O.Ang, Lin - O.Lin }; }
	FSpatialVecx8 operator*(__m256 S) const { return { Ang * S, Lin * S }; }

	static __m256 Dot(const FSpatialVecx8& A, const FSpatialVecx8& B)
	{
		return _mm256_add_ps(FVec3x8::Dot(A.Ang, B.Ang), FVec3x8::Dot(A.Lin, B.Lin));
	}
};

FORCEINLINE FSpatialVecx8 SpatialCrossForce(const FSpatialVecx8& V, const FSpatialVecx8& F)
{
	return {
		FVec3x8::Cross(V.Ang, F.Ang) + FVec3x8::Cross(V.Lin, F.Lin),
		FVec3x8::Cross(V.Ang, F.Lin)
	};
}

FORCEINLINE FSpatialVecx8 TranslateForce(const FSpatialVecx8& F, const FVec3x8& R)
{
	return { F.Ang + FVec3x8::Cross(R, F.Lin), F.Lin };
}

FORCEINLINE FSpatialVecx8 TranslateMotion(const FSpatialVecx8& M, const FVec3x8& R)
{
	return { M.Ang, M.Lin - FVec3x8::Cross(R, M.Ang) };
}

struct FSpatialInertiax8
{
	FMat3x8 Irot;
	FMat3x8 H;
	FMat3x8 MBlock;

	FSpatialInertiax8 operator+(const FSpatialInertiax8& O) const
	{
		return { Irot + O.Irot, H + O.H, MBlock + O.MBlock };
	}

	FSpatialVecx8 Apply(const FSpatialVecx8& V) const
	{
		return { Irot * V.Ang + H * V.Lin, H.Transpose() * V.Ang + MBlock * V.Lin };
	}

	static FSpatialInertiax8 FromRigidBody(__m256 Mass, const FMat3x8& IrotAboutRef, const FMat3x8& HAboutRef)
	{
		FSpatialInertiax8 R;
		R.Irot = IrotAboutRef;
		R.H = HAboutRef;
		R.MBlock = FMat3x8::Identity() * Mass;
		return R;
	}

	/** See FSpatialInertia::TranslatedTo — same derivation, batched across envs. */
	FSpatialInertiax8 TranslatedTo(const FVec3x8& R) const
	{
		const FMat3x8 Rx = FMat3x8::Skew(R);
		FSpatialInertiax8 Out;
		Out.Irot = Irot - (H * Rx) + (Rx * H.Transpose()) - (Rx * MBlock * Rx);
		Out.H = H + Rx * MBlock;
		Out.MBlock = MBlock;
		return Out;
	}
};

/**
 * Storage adapters: each wraps a set of flat per-component TArray<float>
 * (SoA, indexed identically to FCreatureBatchState::BodyIndex) so the
 * solver's scratch accumulators can be loaded/stored as x8 types in the
 * SIMD passes, or read/written one scalar env at a time in the
 * lane-scalar root solve (Pass 3a) — see CreatureBatchSolver.h.
 */
struct FVec3SoA
{
	TArray<float> X, Y, Z;

	void SetNum(int32 N, EAllowShrinking Allow = EAllowShrinking::Yes)
	{
		X.SetNum(N, Allow); Y.SetNum(N, Allow); Z.SetNum(N, Allow);
	}

	FORCEINLINE void Store(int32 Idx, const FVec3x8& V)
	{
		_mm256_storeu_ps(&X[Idx], V.X);
		_mm256_storeu_ps(&Y[Idx], V.Y);
		_mm256_storeu_ps(&Z[Idx], V.Z);
	}

	FORCEINLINE FVec3x8 Load(int32 Idx) const
	{
		return FVec3x8::Load(&X[Idx], &Y[Idx], &Z[Idx]);
	}
};

struct FSpatialVecSoA
{
	TArray<float> AngX, AngY, AngZ, LinX, LinY, LinZ;

	void SetNum(int32 N, EAllowShrinking Allow = EAllowShrinking::Yes)
	{
		AngX.SetNum(N, Allow); AngY.SetNum(N, Allow); AngZ.SetNum(N, Allow);
		LinX.SetNum(N, Allow); LinY.SetNum(N, Allow); LinZ.SetNum(N, Allow);
	}

	FORCEINLINE void Store(int32 Idx, const FSpatialVecx8& V)
	{
		_mm256_storeu_ps(&AngX[Idx], V.Ang.X); _mm256_storeu_ps(&AngY[Idx], V.Ang.Y); _mm256_storeu_ps(&AngZ[Idx], V.Ang.Z);
		_mm256_storeu_ps(&LinX[Idx], V.Lin.X); _mm256_storeu_ps(&LinY[Idx], V.Lin.Y); _mm256_storeu_ps(&LinZ[Idx], V.Lin.Z);
	}

	FORCEINLINE FSpatialVecx8 Load(int32 Idx) const
	{
		return { FVec3x8::Load(&AngX[Idx], &AngY[Idx], &AngZ[Idx]), FVec3x8::Load(&LinX[Idx], &LinY[Idx], &LinZ[Idx]) };
	}

	FORCEINLINE void Accumulate(int32 Idx, const FSpatialVecx8& V) { Store(Idx, Load(Idx) + V); }

	/** Scalar single-env accessors, used by the lane-scalar root solve (Pass 3a) and ball joints (Pass 2/3b). */
	FORCEINLINE FSpatialVec LoadScalar(int32 Idx) const
	{
		return { FVector(AngX[Idx], AngY[Idx], AngZ[Idx]), FVector(LinX[Idx], LinY[Idx], LinZ[Idx]) };
	}

	FORCEINLINE void StoreScalar(int32 Idx, const FSpatialVec& V)
	{
		AngX[Idx] = (float)V.Ang.X; AngY[Idx] = (float)V.Ang.Y; AngZ[Idx] = (float)V.Ang.Z;
		LinX[Idx] = (float)V.Lin.X; LinY[Idx] = (float)V.Lin.Y; LinZ[Idx] = (float)V.Lin.Z;
	}

	/** Read-modify-write a single lane (one env) — lets ball joints' lane-scalar Pass 2 accumulate into one lane of the parent's slot without touching the other 7. */
	FORCEINLINE void AccumulateScalar(int32 Idx, const FSpatialVec& V)
	{
		StoreScalar(Idx, LoadScalar(Idx) + V);
	}
};

struct FSpatialInertiaSoA
{
	TArray<float> Irot[3][3];
	TArray<float> H[3][3];
	TArray<float> MBlock[3][3];

	void SetNum(int32 N, EAllowShrinking Allow = EAllowShrinking::Yes)
	{
		for (int32 r = 0; r < 3; ++r)
		{
			for (int32 c = 0; c < 3; ++c)
			{
				Irot[r][c].SetNum(N, Allow);
				H[r][c].SetNum(N, Allow);
				MBlock[r][c].SetNum(N, Allow);
			}
		}
	}

	FORCEINLINE void Store(int32 Idx, const FSpatialInertiax8& V)
	{
		for (int32 r = 0; r < 3; ++r)
		{
			for (int32 c = 0; c < 3; ++c)
			{
				_mm256_storeu_ps(&Irot[r][c][Idx], V.Irot.M[r][c]);
				_mm256_storeu_ps(&H[r][c][Idx], V.H.M[r][c]);
				_mm256_storeu_ps(&MBlock[r][c][Idx], V.MBlock.M[r][c]);
			}
		}
	}

	FORCEINLINE FSpatialInertiax8 Load(int32 Idx) const
	{
		FSpatialInertiax8 R;
		for (int32 r = 0; r < 3; ++r)
		{
			for (int32 c = 0; c < 3; ++c)
			{
				R.Irot.M[r][c] = _mm256_loadu_ps(&Irot[r][c][Idx]);
				R.H.M[r][c] = _mm256_loadu_ps(&H[r][c][Idx]);
				R.MBlock.M[r][c] = _mm256_loadu_ps(&MBlock[r][c][Idx]);
			}
		}
		return R;
	}

	FORCEINLINE void Accumulate(int32 Idx, const FSpatialInertiax8& V) { Store(Idx, Load(Idx) + V); }

	/** Scalar single-env accessor, used by the lane-scalar root solve (Pass 3a) and ball joints (Pass 2/3b). */
	FORCEINLINE FSpatialInertia LoadScalar(int32 Idx) const
	{
		FSpatialInertia R;
		for (int32 r = 0; r < 3; ++r)
		{
			for (int32 c = 0; c < 3; ++c)
			{
				R.Irot.M[r][c] = Irot[r][c][Idx];
				R.H.M[r][c] = H[r][c][Idx];
				R.MBlock.M[r][c] = MBlock[r][c][Idx];
			}
		}
		return R;
	}

	FORCEINLINE void StoreScalar(int32 Idx, const FSpatialInertia& V)
	{
		for (int32 r = 0; r < 3; ++r)
		{
			for (int32 c = 0; c < 3; ++c)
			{
				Irot[r][c][Idx] = V.Irot.M[r][c];
				H[r][c][Idx] = V.H.M[r][c];
				MBlock[r][c][Idx] = V.MBlock.M[r][c];
			}
		}
	}

	/** Read-modify-write a single lane (one env) — lets ball joints' lane-scalar Pass 2 accumulate into one lane of the parent's slot without touching the other 7. */
	FORCEINLINE void AccumulateScalar(int32 Idx, const FSpatialInertia& V)
	{
		StoreScalar(Idx, LoadScalar(Idx) + V);
	}
};
