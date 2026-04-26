#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>
#include <OpenImageDenoise/oidn.hpp>

class RayTracer
{
public:
	std::vector<float> colorBuffer;
	std::vector<float> albedoBuffer;
	std::vector<float> normalBuffer;
	std::vector<float> outputBuffer;
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	std::thread **threads;
	int numProcs;
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		//film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new MitchellNetravaliFilter());
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread*[numProcs];
		samplers = new MTRandom[numProcs];
		clear();
		int width = (int)scene->camera.width;
		int height = (int)scene->camera.height;
		colorBuffer.resize(width * height * 3);
		albedoBuffer.resize(width * height * 3);
		normalBuffer.resize(width * height * 3);
		outputBuffer.resize(width * height * 3);
	}
	void clear()
	{
		film->clear();
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Compute direct lighting here
		float lightPMF = 0;
		Light* light = scene->sampleLight(sampler, lightPMF);
		if (light == NULL || lightPMF <= 0)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		Colour emitted(0.0f, 0.0f, 0.0f);
		float lightPDF = 0;
		Vec3 lightSample = light->sample(shadingData, sampler, emitted, lightPDF);
		if (lightPDF <= 0)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		if (light->isArea())
		{
			Vec3 xLight = lightSample;
			Vec3 d = xLight - shadingData.x;
			float dist2 = Dot(d, d);
			if (dist2 <= 0)
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}
			Vec3 wi = d.normalize();
			float cos = Dot(shadingData.sNormal, wi);
			if (cos <= 0.0f)
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}
			Vec3 lightNormal = light->normal(shadingData, wi);
			float cosLight = Dot(lightNormal, -wi);
			if (cosLight <= 0)
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}
			if (!scene->visible(shadingData.x, xLight))
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}
			Colour f = shadingData.bsdf->evaluate(shadingData, wi);
			float G = (cos * cosLight) / dist2;
			return (f * emitted) * (G / (lightPMF * lightPDF));
		}
		else
		{
			Vec3 wi = lightSample;
			float cos = Dot(shadingData.sNormal, wi);
			if (cos <= 0)
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}
			Vec3 farPoint =	shadingData.x + wi * ((scene->bounds.max - scene->bounds.min).length() * 2.0f);
			if (!scene->visible(shadingData.x, farPoint))
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}
			Colour f = shadingData.bsdf->evaluate(shadingData, wi);
			return (f * emitted) * (cos / (lightPMF * lightPDF));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour computeDirectMIS(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Compute direct lighting here
		float lightPMF = 0;
		Light* light = scene->sampleLight(sampler, lightPMF);
		if (light == NULL || lightPMF <= 0)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		Colour emitted(0.0f, 0.0f, 0.0f);
		float lightPDF = 0;
		Vec3 lightSample = light->sample(shadingData, sampler, emitted, lightPDF);
		if (lightPDF <= 0 || emitted.Lum() <= 0)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		if (light->isArea())
		{
			Vec3 xLight = lightSample;
			Vec3 d = xLight - shadingData.x;
			float dist2 = Dot(d, d);
			if (dist2 <= 0)
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}
			Vec3 wi = d.normalize();
			float cosSurface = Dot(shadingData.sNormal, wi);
			if (cosSurface <= 0)
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}
			Vec3 lightNormal = light->normal(shadingData, wi);
			float cosLight = Dot(lightNormal, -wi);
			if (cosLight <= 0)
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}
			if (!scene->visible(shadingData.x, xLight))
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}
			Colour f = shadingData.bsdf->evaluate(shadingData, wi);
			float G = (cosSurface * cosLight) / dist2;
			float p_A = lightPMF * lightPDF;
			float p_o = shadingData.bsdf->PDF(shadingData, wi);
			float pbsdf_A = p_o * G / cosSurface;
			float wLight = computeMISWeight(p_A, pbsdf_A);
			return (f * emitted) * G * (wLight / p_A);
		}
		else
		{
			Vec3 wi = lightSample;
			float cosSurface = Dot(shadingData.sNormal, wi);
			if (cosSurface <= 0)
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}
			Vec3 farPoint = shadingData.x + wi * ((scene->bounds.max - scene->bounds.min).length() * 2.0f);
			if (!scene->visible(shadingData.x, farPoint))
			{
				return Colour(0.0f, 0.0f, 0.0f);
			}
			Colour f = shadingData.bsdf->evaluate(shadingData, wi);
			float p_o = lightPMF * lightPDF;
			float pbsdf_o = shadingData.bsdf->PDF(shadingData, wi);
			float wLight = computeMISWeight(p_o, pbsdf_o);
			return (f * emitted) * cosSurface * (wLight / p_o);
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	bool computeIndirectAndHitLight(Colour& col,Colour throughput, Ray& ray, ShadingData shadingData,float pdf)
	{
		IntersectionData nextIntersection = scene->traverse(ray);
		if (nextIntersection.t < FLT_MAX)
		{
			ShadingData nextShadingData = scene->calculateShadingData(nextIntersection, ray);
			if (nextShadingData.bsdf != NULL && nextShadingData.bsdf->isLight())
			{
				Colour Le = nextShadingData.bsdf->emit(nextShadingData, nextShadingData.wo);
				float wIndirect = 1;
				if (!shadingData.bsdf->isPureSpecular())
				{
					Vec3 xLight = nextShadingData.x;
					Vec3 d = xLight - shadingData.x;
					float dist2 = Dot(d, d);
					if (dist2 > 0)
					{
						Vec3 dir = d.normalize();
						float cosSurface = Dot(shadingData.sNormal, dir);
						float cosLight = Dot(nextShadingData.sNormal, -dir);
						if (cosSurface > 0.0f && cosLight > 0.0f)
						{
							float G = (cosSurface * cosLight) / dist2;
							float pbsdf_A = pdf * G / cosSurface;
							float p_A = 0;
							if (!scene->lights.empty())
							{
								Triangle& lightTri = scene->triangles[nextIntersection.ID];
								if (lightTri.area > 0)
								{
									float lightPMF = 1.0f / (float)scene->lights.size();
									p_A = lightPMF * (1.0f / lightTri.area);
								}
							}
							wIndirect = computeMISWeight(pbsdf_A, p_A);
						}
					}
				}
				Colour color = color + throughput * Le * wIndirect;
				return true;
			}
		}
		return false;
	}
	float computeMISWeight(float pa, float pb)
	{
		if (pa + pb < 0) {
			return 0;
		}
		return pa / (pa+pb);
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		// Add pathtracer code here
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t == FLT_MAX)
		{
			if (scene->background != NULL)
			{
				return pathThroughput * scene->background->evaluate(r.dir);
			}
			return Colour(0.0f, 0.0f, 0.0f);
		}
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		Colour color(0.0f, 0.0f, 0.0f);
		if (shadingData.bsdf->isLight())
		{
			if (depth == 0)
			{
				color = color + pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
		}
		//color = color + pathThroughput * computeDirect(shadingData, sampler);
		color = color + pathThroughput * computeDirectMIS(shadingData, sampler);
		if (depth >= 3)
		{
			float pCont = std::min(0.95f, pathThroughput.Lum());
			if (pCont <= 0.0f)
			{
				return color;
			}
			if (sampler->next() > pCont)
			{
				return color;
			}
			pathThroughput = pathThroughput * (1.0f / pCont);
		}
		Colour reflectedColour(0.0f, 0.0f, 0.0f);
		float pdf = 0;
		Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, reflectedColour, pdf);
		if (pdf <= 0)
		{
			return color;
		}
		if (reflectedColour.Lum() <= 0)
		{
			return color;
		}
		Ray nextRay;
		nextRay.init(shadingData.x + wi * EPSILON, wi);
		Colour throughput;
		if (shadingData.bsdf->isPureSpecular())
		{
			throughput = pathThroughput * reflectedColour * (1.0f / pdf);
		}
		else
		{
			float cosTheta = Dot(shadingData.sNormal, wi);
			if (cosTheta <= 0)
			{
				return color;
			}
			throughput = pathThroughput * reflectedColour * (cosTheta / pdf);
		}
		//MIS
		if (computeIndirectAndHitLight(color,throughput, nextRay, shadingData, pdf))
		{
			return color;
		}
		color = color + pathTrace(nextRay, throughput, depth + 1, sampler);
		return color;
	}
	void connectToCamera(Vec3 p, Vec3 n, Colour col)
	{
		float x, y;
		if (!scene->camera.projectOntoCamera(p, x, y))
		{
			return;
		}
		Vec3 toCamera = scene->camera.origin - p;
		float dist2 = Dot(toCamera, toCamera);
		if (dist2 <= 0)
		{
			return;
		}
		toCamera = toCamera.normalize();
		float cosSurface = Dot(n, toCamera);
		if (cosSurface <= 0)
		{
			return;
		}
		Vec3 cameraToPoint = -toCamera;
		float cosCamera = Dot(scene->camera.viewDirection, cameraToPoint);
		if (cosCamera <= 0)
		{
			return;
		}
		if (!scene->visible(p, scene->camera.origin))
		{
			return;
		}
		float We = 1.0f / (scene->camera.Afilm * cosCamera * cosCamera * cosCamera * cosCamera);
		float G = (cosSurface * cosCamera) / dist2;
		film->splat(x, y, col * G * We);		
	}
	void lightTracePath(Ray& r, Colour pathThroughput, Sampler* sampler, int depth)
	{
		if (depth > 8)
		{
			return;
		}
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t == FLT_MAX)
		{
			return;
		}
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (!shadingData.bsdf->isPureSpecular())
		{
			Vec3 wiCamera = scene->camera.origin - shadingData.x;
			wiCamera = wiCamera.normalize();
			Colour f = shadingData.bsdf->evaluate(shadingData, wiCamera);
			connectToCamera(shadingData.x,shadingData.sNormal,pathThroughput * f);
		}
		if (depth >= 3)
		{
			float pCont = std::min(0.95f, pathThroughput.Lum());
			if (pCont <= 0)
			{
				return;
			}
			if (sampler->next() > pCont)
			{
				return;
			}
			pathThroughput = pathThroughput * (1.0f / pCont);
		}
		Colour reflectedColour(0.0f, 0.0f, 0.0f);
		float pdf = 0;
		Vec3 wi = shadingData.bsdf->sample(shadingData,sampler,reflectedColour,pdf);
		if (pdf <= 0 || reflectedColour.Lum() <= 0)
		{
			return;
		}
		Colour throughput;
		if (shadingData.bsdf->isPureSpecular())
		{
			throughput = pathThroughput * reflectedColour * (1.0f / pdf);
		}
		else
		{
			float cosTheta = Dot(shadingData.sNormal, wi);
			if (cosTheta <= 0)
			{
				return;
			}
			throughput = pathThroughput * reflectedColour * (cosTheta / pdf);
		}
		Ray nextRay;
		nextRay.init(shadingData.x + wi * EPSILON, wi);
		lightTracePath(nextRay, throughput, sampler, depth + 1);
	}
	void lightTrace(Sampler* sampler)
	{
		float lightPMF = 0;
		Light* light = scene->sampleLight(sampler, lightPMF);
		if (light == NULL || lightPMF <= 0)
		{
			return;
		}
		float pdfPosition = 0;
		Vec3 p = light->samplePositionFromLight(sampler, pdfPosition);
		if (pdfPosition <= 0)
		{
			return;
		}
		float pdfDirection = 0;
		Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);
		if (pdfDirection <= 0)
		{
			return;
		}
		Colour Le = light->evaluate(-wi);
		if (Le.Lum() <= 0)
		{
			return;
		}
		ShadingData lightSD(p, Vec3(0, 1, 0));
		Vec3 n = light->normal(lightSD, wi);
		float cosLight = Dot(n, wi);
		if (cosLight <= 0)
		{
			return;
		}
		Colour pathThroughput =	Le * (cosLight / (lightPMF * pdfPosition * pdfDirection));
		connectToCamera(p, n, pathThroughput);
		Ray ray;
		ray.init(p + wi * EPSILON, wi);
		lightTracePath(ray, pathThroughput, sampler, 0);
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		// Compute direct lighting for an image sampler here
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return scene->background->evaluate(r.dir);
	}
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	void renderST()
	{
		film->incrementSPP();
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				Colour col = direct(ray, samplers);
				Colour throughput(1.0f, 1.0f, 1.0f);
				col = pathTrace(ray, throughput, 0, samplers);
				//Colour col = viewNormals(ray);
				//Colour col = albedo(ray);
				film->splat(px, py, col);
				//unsigned char r = (unsigned char)(col.r * 255);
				//unsigned char g = (unsigned char)(col.g * 255);
				//unsigned char b = (unsigned char)(col.b * 255);
				//film->tonemap(x, y, r, g, b,4.0f);
				//canvas->draw(x, y, r, g, b);
			}
		}
	}
	void renderWorker(unsigned int yStart, unsigned int yEnd, int threadID)
	{
		Sampler* sampler = &samplers[threadID];
		lightTrace(sampler);
		for (unsigned int y = yStart; y < yEnd; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				Colour throughput(1.0f, 1.0f, 1.0f);
				Colour col = pathTrace(ray, throughput, 0, sampler);
				film->splat(px, py, col);
			}
		}
	}
	void renderMT() 
	{
		film->incrementSPP();
		int threadCount = std::thread::hardware_concurrency();
		if (threadCount == 0) {
			threadCount = 4;
		}
		threadCount = std::min(threadCount, 11);
		threadCount = std::min(threadCount, (int)film->height);
		std::vector<std::thread> threads;
		threads.reserve(threadCount);
		int rowsPerThread = film->height / threadCount;
		int extraRows = film->height % threadCount;
		int currentY = 0;
		for (int i = 0; i < threadCount; i++)
		{
			int yStart = currentY;
			int yEnd = yStart + rowsPerThread;
			if (i < extraRows) {
				yEnd = yStart + rowsPerThread + 1;
			}
			currentY = yEnd;
			threads.emplace_back(&RayTracer::renderWorker,this,yStart,yEnd,i);
		}
		for (auto& t : threads)
		{
			t.join();
		}
	}
	void render_LT()
	{
		film->incrementSPP();
		int sampleCount = film->width * film->height;
		for (int i = 0; i < sampleCount; i++)
		{
			lightTrace(samplers);
		}
	}
	void render()
	{
		bool deNoise = true;
		//renderST();
		renderMT();
		//render_LT();

		if (deNoise) {
			for (unsigned int y = 0; y < film->height; y++)
			{
				for (unsigned int x = 0; x < film->width; x++)
				{
					float px = x + 0.5f;
					float py = y + 0.5f;
					int index = (y * film->width + x) * 3;
					Colour col = film->film[y * film->width + x];
					if (film->SPP > 0) {
						col = col / (float)film->SPP;
					}
					colorBuffer[index + 0] = col.r;
					colorBuffer[index + 1] = col.g;
					colorBuffer[index + 2] = col.b;
					Ray albedoRay = scene->camera.generateRay(px, py);
					Colour alb = albedo(albedoRay);
					albedoBuffer[index + 0] = alb.r;
					albedoBuffer[index + 1] = alb.g;
					albedoBuffer[index + 2] = alb.b;
					Ray normalRay = scene->camera.generateRay(px, py);
					Colour nrm = viewNormals(normalRay);
					normalBuffer[index + 0] = nrm.r;
					normalBuffer[index + 1] = nrm.g;
					normalBuffer[index + 2] = nrm.b;
				}
			}
			denoise();
		}	
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++) {
				unsigned char r, g, b;
				if (deNoise) {
					int index = (y * film->width + x) * 3;
					Colour col(outputBuffer[index + 0], outputBuffer[index + 1], outputBuffer[index + 2]);
					denoisetonemap(col, r, g, b, 4.0f);
				}
				else
				{
					film->tonemap(x, y, r, g, b, 4.0f);
				}				
				canvas->draw(x, y, r, g, b);
			}
		}
	}
	void denoise()
	{
		oidn::DeviceRef device = oidn::newDevice(oidn::DeviceType::CPU);
		device.commit();
		oidn::FilterRef filter = device.newFilter("RT");
		filter.setImage("color", colorBuffer.data(), oidn::Format::Float3, film->width, film->height);
		filter.setImage("albedo", albedoBuffer.data(), oidn::Format::Float3, film->width, film->height);
		filter.setImage("normal", normalBuffer.data(), oidn::Format::Float3, film->width, film->height);
		filter.setImage("output", outputBuffer.data(), oidn::Format::Float3, film->width, film->height);
		filter.set("hdr", true);
		filter.commit();
		filter.execute();
	}
	void denoisetonemap(Colour& color, unsigned char& r, unsigned char& g, unsigned char& b, float exposure) {
		Colour c = color;
		c = c * exposure;
		c.r = c.r / (1.0f + c.r);
		c.g = c.g / (1.0f + c.g);
		c.b = c.b / (1.0f + c.b);
		c.r = powf(fmaxf(c.r, 0.0f), 1.0f / 2.2f);
		c.g = powf(fmaxf(c.g, 0.0f), 1.0f / 2.2f);
		c.b = powf(fmaxf(c.b, 0.0f), 1.0f / 2.2f);
		r = (unsigned char)(c.r * 255.0f);
		g = (unsigned char)(c.g * 255.0f);
		b = (unsigned char)(c.b * 255.0f);
	}
	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};