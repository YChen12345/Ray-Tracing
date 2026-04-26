#pragma once

#include "Core.h"
#include "Geometry.h"
#include "Materials.h"
#include "Sampling.h"

#pragma warning( disable : 4244)

class SceneBounds
{
public:
	Vec3 sceneCentre;
	float sceneRadius;
};

class Light
{
public:
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf) = 0;
	virtual Colour evaluate(const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isArea() = 0;
	virtual Vec3 normal(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float totalIntegratedPower() = 0;
	virtual Vec3 samplePositionFromLight(Sampler* sampler, float& pdf) = 0;
	virtual Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf) = 0;
};

class AreaLight : public Light
{
public:
	Triangle* triangle = NULL;
	Colour emission;
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& emittedColour, float& pdf)
	{
		emittedColour = emission;
		return triangle->sample(sampler, pdf);
	}
	Colour evaluate(const Vec3& wi)
	{
		if (Dot(wi, triangle->gNormal()) < 0)
		{
			return emission;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return 1.0f / triangle->area;
	}
	bool isArea()
	{
		return true;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return triangle->gNormal();
	}
	float totalIntegratedPower()
	{
		return (triangle->area * emission.Lum());
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		return triangle->sample(sampler, pdf);
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Add code to sample a direction from the light
		Vec3 wiLocal = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::cosineHemispherePDF(wiLocal);
		Frame frame;
		frame.fromVector(triangle->gNormal());
		return frame.toWorld(wiLocal);
	}
};

class BackgroundColour : public Light
{
public:
	Colour emission;
	BackgroundColour(Colour _emission)
	{
		emission = _emission;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		reflectedColour = emission;
		return wi;
	}
	Colour evaluate(const Vec3& wi)
	{
		return emission;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		return SamplingDistributions::uniformSpherePDF(wi);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		return emission.Lum() * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 4 * M_PI * use<SceneBounds>().sceneRadius * use<SceneBounds>().sceneRadius;
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		Vec3 wi = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		pdf = SamplingDistributions::uniformSpherePDF(wi);
		return wi;
	}
};

class EnvironmentMap : public Light
{
public:
	Texture* env;
	std::vector<float> cdf;
	std::vector<float> pdfT;
	float totalWeight = 0.0f;
	EnvironmentMap(Texture* _env)
	{
		env = _env;
	}
	void buildDistribution()
	{
		int w = env->width;
		int h = env->height;
		int count = w * h;

		cdf.resize(count);
		pdfT.resize(count);

		totalWeight = 0.0f;

		for (int y = 0; y < h; y++)
		{
			float v = ((float)y + 0.5f) / (float)h;
			float theta = v * M_PI;
			float sinTheta = sinf(theta);

			for (int x = 0; x < w; x++)
			{
				int index = y * w + x;

				float lum = env->texels[index].Lum();

				float weight = lum * sinTheta;

				pdfT[index] = weight;
				totalWeight += weight;
				cdf[index] = totalWeight;
			}
		}

		if (totalWeight > 0.0f)
		{
			for (int i = 0; i < count; i++)
			{
				pdfT[i] /= totalWeight;
				cdf[i] /= totalWeight;
			}
		}
		else
		{
			for (int i = 0; i < count; i++)
			{
				pdfT[i] = 1.0f / (float)count;
				cdf[i] = (float)(i + 1) / (float)count;
			}
		}
		cdf[count - 1] = 1.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Assignment: Update this code to importance sampling lighting based on luminance of each pixel
		float r = sampler->next();
		int count = env->width * env->height;
		int index = 0;
		while (index < count - 1 && cdf[index] < r)
		{
			index++;
		}
		int x = index % env->width;
		int y = index / env->width;
		float u = ((float)x + sampler->next()) / (float)env->width;
		float v = ((float)y + sampler->next()) / (float)env->height;
		float phi = u * 2.0f * M_PI;
		float theta = v * M_PI;
		float sinTheta = sinf(theta);
		Vec3 wi;
		wi.x = sinTheta * cosf(phi);
		wi.y = cosf(theta);
		wi.z = sinTheta * sinf(phi);
		reflectedColour = evaluate(wi);
		pdf = PDF(shadingData, wi);
		return wi;
	}

	Colour evaluate(const Vec3& wi)
	{
		float u = atan2f(wi.z, wi.x);
		u = (u < 0.0f) ? u + (2.0f * M_PI) : u;
		u = u / (2.0f * M_PI);
		float v = acosf(wi.y) / M_PI;
		return env->sample(u, v);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Assignment: Update this code to return the correct PDF of luminance weighted importance sampling
		if (totalWeight <= 0.0f)
		{
			return SamplingDistributions::uniformSpherePDF(wi);
		}
		float u = atan2f(wi.z, wi.x);
		if (u < 0) {
			u = u + (2.0f * M_PI);
		}
		u = u / (2.0f * M_PI);
		float v = acosf(std::max(-1.0f, std::min(1.0f, wi.y))) / M_PI;
		int x = std::min(env->width - 1, std::max(0, (int)(u * env->width)));
		int y = std::min(env->height - 1, std::max(0, (int)(v * env->height)));
		int index = y * env->width + x;
		float theta = v * M_PI;
		float sinTheta = sinf(theta);
		if (sinTheta <= 0)
		{
			return 0.0f;
		}
		float pPixel = pdfT[index];
		return pPixel * (float)(env->width * env->height) / (2.0f * M_PI * M_PI * sinTheta);
	}
	bool isArea()
	{
		return false;
	}
	Vec3 normal(const ShadingData& shadingData, const Vec3& wi)
	{
		return -wi;
	}
	float totalIntegratedPower()
	{
		float total = 0;
		for (int i = 0; i < env->height; i++)
		{
			float st = sinf(((float)i / (float)env->height) * M_PI);
			for (int n = 0; n < env->width; n++)
			{
				total += (env->texels[(i * env->width) + n].Lum() * st);
			}
		}
		total = total / (float)(env->width * env->height);
		return total * 4.0f * M_PI;
	}
	Vec3 samplePositionFromLight(Sampler* sampler, float& pdf)
	{
		// Samples a point on the bounding sphere of the scene. Feel free to improve this.
		Vec3 p = SamplingDistributions::uniformSampleSphere(sampler->next(), sampler->next());
		p = p * use<SceneBounds>().sceneRadius;
		p = p + use<SceneBounds>().sceneCentre;
		pdf = 1.0f / (4 * M_PI * SQ(use<SceneBounds>().sceneRadius));
		return p;
	}
	Vec3 sampleDirectionFromLight(Sampler* sampler, float& pdf)
	{
		// Replace this tabulated sampling of environment maps
		Colour c;
		ShadingData sd;
		return sample(sd, sampler, c, pdf);
	}
};