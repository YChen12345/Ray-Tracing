#pragma once

#include "Core.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define __STDC_LIB_EXT1__
#include "stb_image_write.h"

// Stop warnings about buffer overruns if size is zero. Size should never be zero and if it is the code handles it.
#pragma warning( disable : 6386)

constexpr float texelScale = 1.0f / 255.0f;

class Texture
{
public:
	Colour* texels;
	float* alpha;
	int width;
	int height;
	int channels;
	void loadDefault()
	{
		width = 1;
		height = 1;
		channels = 3;
		texels = new Colour[1];
		texels[0] = Colour(1.0f, 1.0f, 1.0f);
	}
	void load(std::string filename)
	{
		alpha = NULL;
		if (filename.find(".hdr") != std::string::npos)
		{
			float* textureData = stbi_loadf(filename.c_str(), &width, &height, &channels, 0);
			if (width == 0 || height == 0)
			{
				loadDefault();
				return;
			}
			texels = new Colour[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				texels[i] = Colour(textureData[i * channels], textureData[(i * channels) + 1], textureData[(i * channels) + 2]);
			}
			stbi_image_free(textureData);
			return;
		}
		unsigned char* textureData = stbi_load(filename.c_str(), &width, &height, &channels, 0);
		if (width == 0 || height == 0)
		{
			loadDefault();
			return;
		}
		texels = new Colour[width * height];
		for (int i = 0; i < (width * height); i++)
		{
			texels[i] = Colour(textureData[i * channels] / 255.0f, textureData[(i * channels) + 1] / 255.0f, textureData[(i * channels) + 2] / 255.0f);
		}
		if (channels == 4)
		{
			alpha = new float[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				alpha[i] = textureData[(i * channels) + 3] / 255.0f;
			}
		}
		stbi_image_free(textureData);
	}
	Colour sample(const float tu, const float tv) const
	{
		Colour tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		Colour s[4];
		s[0] = texels[y * width + x];
		s[1] = texels[y * width + ((x + 1) % width)];
		s[2] = texels[((y + 1) % height) * width + x];
		s[3] = texels[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	float sampleAlpha(const float tu, const float tv) const
	{
		if (alpha == NULL)
		{
			return 1.0f;
		}
		float tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		float s[4];
		s[0] = alpha[y * width + x];
		s[1] = alpha[y * width + ((x + 1) % width)];
		s[2] = alpha[((y + 1) % height) * width + x];
		s[3] = alpha[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	~Texture()
	{
		delete[] texels;
		if (alpha != NULL)
		{
			delete alpha;
		}
	}
};

class ImageFilter
{
public:
	virtual float filter(const float x, const float y) const = 0;
	virtual int size() const = 0;
};

class BoxFilter : public ImageFilter
{
public:
	float filter(float x, float y) const
	{
		if (fabsf(x) <= 0.5f && fabs(y) <= 0.5f)
		{
			return 1.0f;
		}
		return 0;
	}
	int size() const
	{
		return 0;
	}
};

class GaussianFilter : public ImageFilter
{
public:
	int radius = 2;
	float G(float d,float r) const
	{
		float alpha = 2;
		return fmaxf(0.0f, expf(-alpha * d * d) - expf(-alpha * r * r));
	}

	float filter(float x, float y) const override
	{
		if (fabsf(x) >= radius || fabsf(y) >= radius)
		{
			return 0;
		}
		return G(x,radius) * G(y,radius);
	}

	int size() const override
	{
		return radius;
	}
};

class MitchellNetravaliFilter : public ImageFilter
{
public:
	float B = 1.0f/3.0f;
	float C = 1.0f/3.0f;

	float hm(float x) const
	{
		if (fabsf(x) >= 2) 
		{
			return 0;
		}
		else if (fabsf(x) >= 1)
		{
			return (1.0f / 6.0f) * ((-B - 6 * C) * fabsf(x * x * x) + (6* B + 30 * C) * fabsf(x * x) + (-12*B-48*C)* fabsf(x)+(8*B+24*C));
		}
		else
		{
			return (1.0f / 6.0f) * ((12 - 9 * B - 6 * C) * fabsf(x * x * x) + (-18 + 12 * B + 6 * C) * fabsf(x * x) + (6 - 2 * B));
		}
	}

	float filter(float x, float y) const override
	{
		return hm(x) * hm(y);
	}

	int size() const override
	{
		return 2;
	}
};

class Film
{
public:
	Colour* film;
	unsigned int width;
	unsigned int height;
	int SPP;
	ImageFilter* filter;
	void splat(const float x, const float y, const Colour& L)
	{
		// Code to splat a smaple with colour L into the image plane using an ImageFilter
		float filterWeights[25]; // Storage to cache weights
		unsigned int indices[25]; // Store indices to minimize computations
		unsigned int used = 0;
		float total = 0;
		int size = filter->size();
		for (int i = -size; i <= size; i++) {
			for (int j = -size; j <= size; j++) {
				int px = (int)x + j;
				int py = (int)y + i;
				if (px >= 0 && px < width && py >= 0 && py < height) {
					indices[used] = (py * width) + px;
					filterWeights[used] = filter->filter(px - x, py - y);
					total += filterWeights[used];
					used++;
				}
			}
		}
		for (int i = 0; i < used; i++) {
			film[indices[i]] = film[indices[i]] + (L * filterWeights[i] / total);
		}
	}
	void tonemapmethod_l(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, float exposure) 
	{
		Colour c = film[y * width + x];
		if (SPP > 0)
		{
			c = c / (float)SPP;
		}
		float p = powf(2, exposure - 1);
		c.r = powf(c.r * p, 1.0f / 2.2f);
		c.g = powf(c.g * p, 1.0f / 2.2f);
		c.b = powf(c.b * p, 1.0f / 2.2f);
		c.r = fminf(1.0f, fmaxf(0.0f, c.r));
		c.g = fminf(1.0f, fmaxf(0.0f, c.g));
		c.b = fminf(1.0f, fmaxf(0.0f, c.b));
		r = (unsigned char)(c.r * 255.0f);
		g = (unsigned char)(c.g * 255.0f);
		b = (unsigned char)(c.b * 255.0f);
	}
	void tonemapmethod_rg(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, float exposure) 
	{
		Colour c = film[y * width + x];
		if (SPP > 0)
		{
			c = c / (float)SPP;
		}
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
	void tonemapmethod_f(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, float exposure)
	{
		Colour c = film[y * width + x];
		if (SPP > 0)
		{
			c = c / (float)SPP;
		}
		c = c * exposure;
		float W = 11.2f;
		float cw = C(W);
		c.r = powf(C(c.r) / cw, 1.0f / 2.2f);
		c.g = powf(C(c.g) / cw, 1.0f / 2.2f);
		c.b = powf(C(c.b) / cw, 1.0f / 2.2f);
		c.r = fminf(1.0f, fmaxf(0.0f, c.r));
		c.g = fminf(1.0f, fmaxf(0.0f, c.g));
		c.b = fminf(1.0f, fmaxf(0.0f, c.b));
		r = (unsigned char)(c.r * 255.0f);
		g = (unsigned char)(c.g * 255.0f);
		b = (unsigned char)(c.b * 255.0f);
	}
	float C(float x) {
		float A = 0.15f;
		float B = 0.50f;
		float C = 0.10f;
		float D = 0.20f;
		float E = 0.02f;
		float F = 0.30f;
		return (x * (A * x + C * B) + D * E) / (x * (A * x + B) + D * F) - E / F;
	}
	void tonemap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, float exposure = 1.0f)
	{
		// Return a tonemapped pixel at coordinates x, y
		//tonemapmethod_l(x, y, r, g, b, exposure);
		//tonemapmethod_rg(x, y, r, g, b, exposure);
		tonemapmethod_f(x, y, r, g, b, exposure);
	}
	// Do not change any code below this line
	void init(int _width, int _height, ImageFilter* _filter)
	{
		width = _width;
		height = _height;
		film = new Colour[width * height];
		clear();
		filter = _filter;
	}
	void clear()
	{
		memset(film, 0, width * height * sizeof(Colour));
		SPP = 0;
	}
	void incrementSPP()
	{
		SPP++;
	}
	void save(std::string filename)
	{
		Colour* hdrpixels = new Colour[width * height];
		for (unsigned int i = 0; i < (width * height); i++)
		{
			hdrpixels[i] = film[i] / (float)SPP;
		}
		stbi_write_hdr(filename.c_str(), width, height, 3, (float*)hdrpixels);
		delete[] hdrpixels;
	}
};