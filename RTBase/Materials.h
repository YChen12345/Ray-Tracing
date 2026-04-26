#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"

#pragma warning( disable : 4244)
#pragma warning( disable : 4305) // Double to float

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper
{
public:
	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt)
	{
		// Add code here
		float etaI = iorExt;
		float etaT = iorInt;
		bool entering = (cosTheta >= 0);
		if (!entering)
		{
			std::swap(etaI, etaT);
			cosTheta = fabsf(cosTheta);
		}
		float sinThetaI = sqrtf(1.0f - cosTheta * cosTheta);
		float sinThetaT = (etaI / etaT) * sinThetaI;
		if (sinThetaT >= 1)
		{
			return 1.0f;
		}
		float cosThetaT = sqrtf(1.0f - sinThetaT * sinThetaT);
		float Rs = ((etaT * cosTheta) - (etaI * cosThetaT)) / ((etaT * cosTheta) + (etaI * cosThetaT));
		float Rp = ((etaI * cosTheta) - (etaT * cosThetaT)) / ((etaI * cosTheta) + (etaT * cosThetaT));
		return 0.5f * (Rs * Rs + Rp * Rp);
	}
	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k)
	{
		// Add code here
		cosTheta = fabsf(cosTheta);
		float cos2 = cosTheta * cosTheta;
		float sin2 = 1.0f - cos2;
		float eta2r = ior.r * ior.r;
		float k2r = k.r * k.r;
		float t0r = eta2r - k2r - sin2;
		float a2pb2r = sqrtf((t0r * t0r + 4.0f * eta2r * k2r));
		float ar = sqrtf(0.5f * (a2pb2r + t0r));
		float t1r = a2pb2r + cos2;
		float t2r = 2.0f * cosTheta * ar;
		float Rsr = (t1r - t2r) / (t1r + t2r);
		float t3r = cos2 * a2pb2r + sin2 * sin2;
		float t4r = t2r * sin2;
		float Rpr = Rsr * ((t3r - t4r) / (t3r + t4r));
		float Fr = 0.5f * (Rsr + Rpr);
		float eta2g = ior.g * ior.g;
		float k2g = k.g * k.g;
		float t0g = eta2g - k2g - sin2;
		float a2pb2g = sqrtf((t0g * t0g + 4.0f * eta2g * k2g));
		float ag = sqrtf((0.5f * (a2pb2g + t0g)));
		float t1g = a2pb2g + cos2;
		float t2g = 2.0f * cosTheta * ag;
		float Rsg = (t1g - t2g) / (t1g + t2g);
		float t3g = cos2 * a2pb2g + sin2 * sin2;
		float t4g = t2g * sin2;
		float Rpg = Rsg * ((t3g - t4g) / (t3g + t4g));
		float Fg = 0.5f * (Rsg + Rpg);
		float eta2b = ior.b * ior.b;
		float k2b = k.b * k.b;
		float t0b = eta2b - k2b - sin2;
		float a2pb2b = sqrtf((t0b * t0b + 4.0f * eta2b * k2b));
		float ab = sqrtf((0.5f * (a2pb2b + t0b)));
		float t1b = a2pb2b + cos2;
		float t2b = 2.0f * cosTheta * ab;
		float Rsb = (t1b - t2b) / (t1b + t2b);
		float t3b = cos2 * a2pb2b + sin2 * sin2;
		float t4b = t2b * sin2;
		float Rpb = Rsb * ((t3b - t4b) / (t3b + t4b));
		float Fb = 0.5f * (Rsb + Rpb);
		return Colour(Fr, Fg, Fb);
	}
	static float lambdaGGX(Vec3 wi, float alpha)
	{
		// Add code here
		float cosTheta = fabsf(wi.z);
		if (cosTheta <= 0)
		{
			return 0;
		}
		float sin2Theta = 1.0f - wi.z * wi.z;
		float tan2Theta = sin2Theta / (cosTheta * cosTheta);
		if (!std::isfinite(tan2Theta))
		{
			return 0;
		}
		float a2 = alpha * alpha;
		return 0.5f * (-1.0f + sqrtf(1.0f + a2 * tan2Theta));
	}
	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		// Add code here
		return 1.0f / (1.0f + lambdaGGX(wi, alpha) + lambdaGGX(wo, alpha));
	}
	static float Dggx(Vec3 h, float alpha)
	{
		// Add code here
		float cosTheta = h.z;
		if (cosTheta <= 0)
		{
			return 0;
		}
		float a2 = alpha * alpha;
		float cos2 = cosTheta * cosTheta;
		float denom = cos2 * (a2 - 1.0f) + 1.0f;
		return a2 / (M_PI * denom * denom);
	}
};

class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;
	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add correct sampling code here
		Vec3 wiLocal = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wiLocal);
		Colour f = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		reflectedColour = f * wiLocal.z;
		return shadingData.frame.toWorld(wiLocal);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add correct PDF code here
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		if (wiLocal.z <= 0)
		{
			return 0;
		}
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Mirror sampling code
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 wiLocal(-woLocal.x, -woLocal.y,woLocal.z);
		pdf = 1.0f;
		reflectedColour = albedo->sample(shadingData.tu,shadingData.tv);
		return shadingData.frame.toWorld(wiLocal);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror evaluation code
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror PDF
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};


class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Conductor sampling code
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (woLocal.z <= 0)
		{
			pdf = 0;
			reflectedColour = Colour(0, 0, 0);
			return Vec3(0, 0, 1);
		}
		float r1 = sampler->next();
		float r2 = sampler->next();
		float a = std::max(alpha,0.03f);
		float a2 = a * a;
		float phi = 2.0f * M_PI * r1;
		float tan2Theta = a2 * r2 / (1.0f - r2);
		float cosTheta = 1.0f / sqrtf(1.0f + tan2Theta);
		float sinTheta = sqrtf(1.0f - cosTheta * cosTheta);
		Vec3 hLocal(sinTheta * cosf(phi),sinTheta * sinf(phi),cosTheta);
		if (Dot(woLocal, hLocal) <= 0)
		{
			pdf = 0;
			reflectedColour = Colour(0, 0, 0);
			return shadingData.frame.toWorld(Vec3(0, 0, 1));
		}
		Vec3 wiLocal = -woLocal + hLocal * (2.0f * Dot(woLocal, hLocal));
		if (wiLocal.z <= 0)
		{
			pdf = 0;
			reflectedColour = Colour(0, 0, 0);
			return shadingData.frame.toWorld(wiLocal);
		}
		float D = ShadingHelper::Dggx(hLocal, a);
		float pdfH = D * hLocal.z;
		pdf = pdfH / 4.0f * Dot(woLocal, hLocal);
		if (pdf <= 0)
		{
			reflectedColour = Colour(0, 0, 0);
			return shadingData.frame.toWorld(wiLocal);
		}
		Colour f = evaluate(shadingData, shadingData.frame.toWorld(wiLocal));
		reflectedColour = f * fabsf(wiLocal.z) / pdf;
		return shadingData.frame.toWorld(wiLocal);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor evaluation code
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (wiLocal.z <= 0 || woLocal.z <= 0)
		{
			return Colour(0, 0, 0);
		}
		Vec3 hLocal = wiLocal + woLocal;
		float hLen2 = Dot(hLocal, hLocal);
		if (hLen2 <= 0)
		{
			return Colour(0, 0, 0);
		}
		hLocal = hLocal * (1.0f / sqrtf(hLen2));
		if (hLocal.z <= 0)
		{
			hLocal = -hLocal;
		}
		float cosI = wiLocal.z;
		float cosO = woLocal.z;
		float a = std::max(alpha, 0.03f);
		float D = ShadingHelper::Dggx(hLocal, a);
		float G = ShadingHelper::Gggx(wiLocal, woLocal, a);
		Colour F = ShadingHelper::fresnelConductor(Dot(wiLocal, hLocal), eta, k);
		Colour base = albedo->sample(shadingData.tu, shadingData.tv);
		return base * F * (D * G / 4.0f * cosI * cosO);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Conductor PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (wiLocal.z <= 0 || woLocal.z <= 0)
		{
			return 0;
		}
		Vec3 hLocal = wiLocal + woLocal;
		float hLen2 = Dot(hLocal, hLocal);
		if (hLen2 <= 0)
		{
			return 0;
		}
		hLocal = hLocal * (1.0f / sqrtf(hLen2));
		if (hLocal.z <= 0)
		{
			hLocal = -hLocal;
		}
		float woDotH = Dot(woLocal, hLocal);
		if (woDotH <= 0)
		{
			return 0;
		}
		float a = std::max(alpha, 0.03f);
		float D = ShadingHelper::Dggx(hLocal, a);
		float pdfH = D * hLocal.z;
		return pdfH / 4.0f * woDotH;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Glass sampling code
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		float etaI = extIOR;
		float etaT = intIOR;
		if (woLocal.z < 0)
		{
			std::swap(etaI, etaT);
		}
		float Fr = ShadingHelper::fresnelDielectric(woLocal.z, intIOR, extIOR);
		Colour baseColour = albedo->sample(shadingData.tu,shadingData.tv);
		float xi = sampler->next();
		if (xi < Fr)
		{
			Vec3 wiLocal(-woLocal.x,-woLocal.y,woLocal.z);
			pdf = Fr;
			reflectedColour = baseColour;
			return shadingData.frame.toWorld(wiLocal);
		}
		float cosThetaO = fabsf(woLocal.z);
		float eta = etaI / etaT;
		float sin2ThetaO = 1.0f - cosThetaO * cosThetaO;
		float sin2ThetaT = eta * eta * sin2ThetaO;
		if (sin2ThetaT >= 1.0f)
		{
			Vec3 wiLocal(-woLocal.x,-woLocal.y,woLocal.z);
			pdf = 1.0f;
			reflectedColour = baseColour;
			return shadingData.frame.toWorld(wiLocal);
		}
		float cosThetaT = sqrtf(1.0f - sin2ThetaT);
		float sign = 1.0f;
		if (woLocal.z > 0) {
			sign = -1.0f;
		}
		Vec3 wtLocal(-eta * woLocal.x, -eta * woLocal.y, sign * cosThetaT);
		wtLocal = wtLocal.normalize();
		pdf = 1.0f - Fr;
		float etaScale = (etaI * etaI) / (etaT * etaT);
		reflectedColour = baseColour * etaScale;
		return shadingData.frame.toWorld(wtLocal);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Glass evaluation code
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with GlassPDF
		return 0.0f;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		//return false;
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (woLocal.z <= 0)
		{
			pdf = 0;
			reflectedColour = Colour(0, 0, 0);
			return Vec3(0, 0, 1);
		}
		float r1 = sampler->next();
		float r2 = sampler->next();
		float phi = 2.0f * M_PI * r1;
		float a2 = alpha * alpha;
		float tan2Theta = a2 * r2 / (1.0f - r2);
		float cosTheta = 1.0f / sqrtf(1.0f + tan2Theta);
		float sinTheta = sqrtf(1.0f - cosTheta * cosTheta);
		Vec3 h(cosf(phi) * sinTheta, sinf(phi) * sinTheta,cosTheta);
		h = h.normalize();
		Vec3 wiLocal = (h * (2.0f * Dot(woLocal, h)) - woLocal).normalize();
		if (wiLocal.z <= 0)
		{
			pdf = 0;
			reflectedColour = Colour(0, 0, 0);
			return Vec3(0, 0, 1);
		}
		float D = ShadingHelper::Dggx(h, alpha);
		float pdfH = D * h.z;
		float woDotH = Dot(woLocal, h);
		pdf = pdfH / (4.0f * woDotH);
		Colour f = evaluate(shadingData, shadingData.frame.toWorld(wiLocal));
		reflectedColour = f * wiLocal.z;
		return shadingData.frame.toWorld(wiLocal);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (wiLocal.z <= 0 || woLocal.z <= 0)
		{
			return Colour(0, 0, 0);
		}
		Vec3 h = (wiLocal + woLocal).normalize();
		if (h.z <= 0)
		{
			return Colour(0, 0, 0);
		}
		float cosI = wiLocal.z;
		float cosO = woLocal.z;
		float D = ShadingHelper::Dggx(h, alpha);
		float G = ShadingHelper::Gggx(wiLocal, woLocal, alpha);
		float F = ShadingHelper::fresnelDielectric(Dot(wiLocal, h),intIOR,extIOR);
		Colour baseColour = albedo->sample(shadingData.tu,shadingData.tv);
		return baseColour * (F * D * G / std::max(4.0f * cosI * cosO, 0.01f));
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (wiLocal.z <= 0 || woLocal.z <= 0)
		{
			return 0;
		}
		Vec3 h = (wiLocal + woLocal).normalize();
		if (h.z <= 0)
		{
			return 0;
		}
		float D = ShadingHelper::Dggx(h, alpha);
		float pdfH = D * h.z;
		float woDotH = Dot(woLocal, h);
		return pdfH / (4.0f * woDotH);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		Vec3 wiLocal = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wiLocal);
		Vec3 wi = shadingData.frame.toWorld(wiLocal);
		Colour f = evaluate(shadingData, wi);
		reflectedColour = f * wiLocal.z;
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (wiLocal.z <= 0 || woLocal.z <= 0)
		{
			return Colour(0, 0, 0);
		}
		float sigma2 = sigma * sigma;
		float A = 1.0f - sigma2 / (2.0f * (sigma2 + 0.33f));
		float B = 0.45f * sigma2 / (sigma2 + 0.09f);
		float sinThetaI = sqrtf(1.0f - wiLocal.z * wiLocal.z);
		float sinThetaO = sqrtf(1.0f - woLocal.z * woLocal.z);
		float maxCos = 0;
		if (sinThetaI > 0 && sinThetaO > 0)
		{
			float cosPhiI = wiLocal.x / sinThetaI;
			float sinPhiI = wiLocal.y / sinThetaI;
			float cosPhiO = woLocal.x / sinThetaO;
			float sinPhiO = woLocal.y / sinThetaO;
			float cosPhiDiff = cosPhiI * cosPhiO + sinPhiI * sinPhiO;
			maxCos = cosPhiDiff;
		}
		float sinAlpha;
		float tanBeta;
		if (fabsf(wiLocal.z) > fabsf(woLocal.z))
		{
			sinAlpha = sinThetaO;
			tanBeta = sinThetaI / fabsf(wiLocal.z);
		}
		else
		{
			sinAlpha = sinThetaI;
			tanBeta = sinThetaO / fabsf(woLocal.z);
		}
		float oren = A + B * maxCos * sinAlpha * tanBeta;
		Colour colour = albedo->sample(shadingData.tu, shadingData.tv);
		return colour * (oren / M_PI);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		if (wiLocal.z <= 0)
		{
			return 0;
		}
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.01f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Plastic sampling code
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (woLocal.z <= 0)
		{
			pdf = 0;
			reflectedColour = Colour(0, 0, 0);
			return Vec3(0, 0, 1);
		}
		float exponent = alphaToPhongExponent();
		float sp = ShadingHelper::fresnelDielectric(woLocal.z, intIOR, extIOR);
		sp = std::max(0.05f, std::min(0.5f, sp * 4.0f));
		Vec3 wiLocal;
		if (sampler->next() < sp)
		{
			Vec3 mirrorLocal(-woLocal.x, -woLocal.y, woLocal.z);
			Frame mirrorFrame;
			mirrorFrame.fromVector(mirrorLocal);
			float u1 = sampler->next();
			float u2 = sampler->next();
			float cosTheta = powf(u1, 1.0f / (exponent + 1.0f));
			float sinTheta = sqrtf(1.0f - cosTheta * cosTheta);
			float phi = 2.0f * M_PI * u2;
			Vec3 lobeLocal(cosf(phi) * sinTheta, sinf(phi) * sinTheta, cosTheta);
			wiLocal = mirrorFrame.toWorld(lobeLocal).normalize();
			if (wiLocal.z <= 0)
			{
				pdf = 0;
				reflectedColour = Colour(0, 0, 0);
				return shadingData.frame.toWorld(Vec3(0, 0, 1));
			}
		}
		else
		{
			wiLocal = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		}
		Vec3 wi = shadingData.frame.toWorld(wiLocal);
		float diffPdf = SamplingDistributions::cosineHemispherePDF(wiLocal);
		Vec3 mirrorLocal(-woLocal.x, -woLocal.y, woLocal.z);
		float specPdf = 0;
		float cosAlpha = Dot(wiLocal.normalize(), mirrorLocal.normalize());
		if (cosAlpha > 0)
		{
			specPdf = ((exponent + 1.0f) / (2.0f * M_PI)) * powf(cosAlpha, exponent);
		}
		pdf = sp * specPdf + (1.0f - sp) * diffPdf;
		Colour f = evaluate(shadingData, wi);
		reflectedColour = f * wiLocal.z;
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic evaluation code
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (wiLocal.z <= 0 || woLocal.z <= 0)
		{
			return Colour(0, 0, 0);
		}
		Colour baseColour = albedo->sample(shadingData.tu, shadingData.tv);
		float F = ShadingHelper::fresnelDielectric(woLocal.z, intIOR, extIOR);
		Colour diffuse = baseColour * ((1.0f - F) / M_PI);
		Vec3 mirrorLocal(-woLocal.x, -woLocal.y, woLocal.z);
		float exponent = alphaToPhongExponent();
		float cosAlpha = Dot(wiLocal.normalize(), mirrorLocal.normalize());
		Colour glossy(0, 0, 0);
		if (cosAlpha > 0)
		{
			float spec = ((exponent + 2.0f) / (2.0f * M_PI)) * powf(cosAlpha, exponent);
			glossy = Colour(1.0f, 1.0f, 1.0f) * (F * spec);
		}
		return diffuse + glossy;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		if (wiLocal.z <= 0 || woLocal.z <= 0)
		{
			return 0;
		}
		float exponent = alphaToPhongExponent();
		float F = ShadingHelper::fresnelDielectric(woLocal.z, intIOR, extIOR);
		float sp = std::max(0.05f, std::min(0.5f, F * 4.0f));
		float diffPdf = SamplingDistributions::cosineHemispherePDF(wiLocal);
		Vec3 mirrorLocal(-woLocal.x, -woLocal.y, woLocal.z);
		float specPdf = 0;
		float cosAlpha = Dot(wiLocal.normalize(), mirrorLocal.normalize());
		if (cosAlpha > 0)
		{
			specPdf = ((exponent + 1.0f) / (2.0f * M_PI)) * powf(cosAlpha, exponent);
		}
		return sp * specPdf + (1.0f - sp) * diffPdf;
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};