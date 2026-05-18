#include <Eigen/Dense>
#include <lodepng.h>
#include <json/json.hpp>

#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include <fstream>
#include <algorithm>
#include <cfloat>

#include "BVHNode.hpp"
#include "Triangle.hpp"
#include "Scene.hpp"
#include "Camera.hpp"
#include "PointLight.hpp"
#include "DirectionalLight.hpp"
#include "LambertianShader.hpp"
#include "TexturedLambertianShader.hpp"
#include "PhongShader.hpp"
#include "MirrorShader.hpp"
#include "TexCoordTestShader.hpp"
#include "Model.hpp"

nlohmann::json static loadConfig(const std::string& filename)
{
	std::ifstream configStream(filename);
	return nlohmann::json::parse(configStream);
}

Eigen::Vector3f static loadVec3FromConfig(const nlohmann::json& config)
{
	return Eigen::Vector3f(config[0], config[1], config[2]);
}

int main(int argc, char* argv[])
{
	auto config = loadConfig("../config/config.json");

	const int pixWidth = config["pixWidth"];
	const int pixHeight = config["pixHeight"];
	const int nChannels = 4;

	Camera cam(
		loadVec3FromConfig(config["cameraPos"]),
		loadVec3FromConfig(config["cameraForward"]),
		loadVec3FromConfig(config["cameraUp"]),
		pixWidth,
		pixHeight,
		config["cameraFov"]
	);

	std::vector<uint8_t> outImage(
		pixWidth * pixHeight * nChannels
	);

	std::cout << "Camera Pos: "
		<< config["cameraPos"] << std::endl;

	std::cout << "Camera Forward: "
		<< config["cameraForward"] << std::endl;

	Eigen::Vector3f red(1.f, 0.f, 0.f);
	Eigen::Vector3f blue(0.f, 0.f, 1.f);
	Eigen::Vector3f aqua(0.f, 0.8f, 0.8f);

	Eigen::Vector3f lavender(
		178.f / 255.f,
		164.f / 255.f,
		212.f / 255.f
	);

	std::vector<uint8_t> archesTexture;
	unsigned int archesWidth, archesHeight;
	lodepng::decode(
		archesTexture,
		archesWidth,
		archesHeight,
		"../models/mur-bas_diffuse.png"
	);

	std::vector<uint8_t> archesInnerTexture;
	unsigned int archesInnerWidth, archesInnerHeight;
	lodepng::decode(
		archesInnerTexture,
		archesInnerWidth,
		archesInnerHeight,
		"../models/arche001_diffuse.png"
	);

	std::vector<uint8_t> archesUpperTexture;
	unsigned int archesUpperWidth, archesUpperHeight;
	lodepng::decode(
		archesUpperTexture,
		archesUpperWidth,
		archesUpperHeight,
		"../models/mur-haut_diffuse.png"
	);

	std::vector<uint8_t> outerRoofTexture;
	unsigned int outerRoofWidth, outerRoofHeight;
	lodepng::decode(
		outerRoofTexture,
		outerRoofWidth,
		outerRoofHeight,
		"../models/arche_diffuse.png"
	);

	std::vector<uint8_t> outerPillarsTexture;
	unsigned int outerPillarsWidth, outerPillarsHeight;
	lodepng::decode(
		outerPillarsTexture,
		outerPillarsWidth,
		outerPillarsHeight,
		"../models/colonne-ext_diffuse.png"
	);

	std::vector<uint8_t> halfPillarsTexture;
	unsigned int halfPillarsWidth, halfPillarsHeight;
	lodepng::decode(
		halfPillarsTexture,
		halfPillarsWidth,
		halfPillarsHeight,
		"../models/colonne-ext_diffuse.png"
	);

	std::vector<uint8_t> windowFrameTexture;
	unsigned int windowFrameWidth, windowFrameHeight;
	lodepng::decode(
		windowFrameTexture,
		windowFrameWidth,
		windowFrameHeight,
		"../models/mur-bas-ext_diffuse.png"
	);

	std::vector<uint8_t> flooringTexture;
	unsigned int flooringWidth, flooringHeight;
	lodepng::decode(
		flooringTexture,
		flooringWidth,
		flooringHeight,
		"../models/material_24_diffuse.png"
	);

	std::vector<uint8_t> roofDetailTexture;
	unsigned int roofDetailWidth, roofDetailHeight;
	lodepng::decode(
		roofDetailTexture,
		roofDetailWidth,
		roofDetailHeight,
		"../models/arche002_diffuse.png"
	);

	std::vector<uint8_t> wallDetailTexture;
	unsigned int wallDetailWidth, wallDetailHeight;
	lodepng::decode(
		wallDetailTexture,
		wallDetailWidth,
		wallDetailHeight,
		"../models/debord-mur_diffuse.png"
	);

	std::vector<uint8_t> pillarBaseTexture;
	unsigned int pillarBaseWidth, pillarBaseHeight;
	lodepng::decode(
		pillarBaseTexture,
		pillarBaseWidth,
		pillarBaseHeight,
		"../models/pied-colonne_diffuse.png"
	);

	std::vector<uint8_t> innerPillarOneTexture;
	unsigned int innerPillarOneWidth, innerPillarOneHeight;
	lodepng::decode(
		innerPillarOneTexture,
		innerPillarOneWidth,
		innerPillarOneHeight,
		"../models/colonne_diffuse.png"
	);

	std::vector<uint8_t> innerPillarTwoTexture;
	unsigned int innerPillarTwoWidth, innerPillarTwoHeight;
	lodepng::decode(
		innerPillarTwoTexture,
		innerPillarTwoWidth,
		innerPillarTwoHeight,
		"../models/colonne_diffuse.png"
	);

	std::vector<uint8_t> altarBaseTexture;
	unsigned int altarBaseWidth, altarBaseHeight;
	lodepng::decode(
		altarBaseTexture,
		altarBaseWidth,
		altarBaseHeight,
		"../models/estrade-contour_specularGlossiness.png"
	);

	std::vector<uint8_t> altarBaseDetailTexture;
	unsigned int altarBaseDetailWidth, altarBaseDetailHeight;
	lodepng::decode(
		altarBaseDetailTexture,
		altarBaseDetailWidth,
		altarBaseDetailHeight,
		"../models/estrade-contour_specularGlossiness.png"
	);

	std::vector<uint8_t> altarTableTexture;
	unsigned int altarTableWidth, altarTableHeight;
	lodepng::decode(
		altarTableTexture,
		altarTableWidth,
		altarTableHeight,
		"../models/autel_specularGlossiness.png"
	);

	std::vector<uint8_t> windowDetailedTexture;
	unsigned int windowDetailedWidth, windowDetailedHeight;
	lodepng::decode(
		windowDetailedTexture,
		windowDetailedWidth,
		windowDetailedHeight,
		"../models/vitrail-fond.005_diffuse.png"
	);

	std::vector<uint8_t> windowBasicTexture;
	unsigned int windowBasicWidth, windowBasicHeight;
	lodepng::decode(
		windowBasicTexture,
		windowBasicWidth,
		windowBasicHeight,
		"../models/vitrail-fond.005_diffuse.png"
	);

	std::vector<uint8_t> windowUpperTexture;
	unsigned int windowUpperWidth, windowUpperHeight;
	lodepng::decode(
		windowUpperTexture,
		windowUpperWidth,
		windowUpperHeight,
		"../models/vitrail-fond.005_diffuse.png"
	);

	std::vector<uint8_t> seatsOneTexture;
	unsigned int seatsOneWidth, seatsOneHeight;
	lodepng::decode(
		seatsOneTexture,
		seatsOneWidth,
		seatsOneHeight,
		"../models/banc_diffuse.png"
	);

	std::vector<uint8_t> seatsTwoTexture;
	unsigned int seatsTwoWidth, seatsTwoHeight;
	lodepng::decode(
		seatsTwoTexture,
		seatsTwoWidth,
		seatsTwoHeight,
		"../models/banc_diffuse.png"
	);

	std::vector<uint8_t> seatsThreeTexture;
	unsigned int seatsThreeWidth, seatsThreeHeight;
	lodepng::decode(
		seatsThreeTexture,
		seatsThreeWidth,
		seatsThreeHeight,
		"../models/banc_diffuse.png"
	);

	LambertianShader redLambertianShader(red);

	PhongShader bluePlasticShader(
		blue,
		Eigen::Vector3f(1.f, 1.f, 1.f),
		100.f
	);

	LambertianShader aquaLambertianShader(aqua);
	LambertianShader lavenderLambertianShader(lavender);

	TexturedLambertianShader archesShader(
		&archesTexture,
		archesWidth,
		archesHeight
	);

	TexturedLambertianShader archesInnerShader(
		&archesInnerTexture,
		archesInnerWidth,
		archesInnerHeight
	);

	TexturedLambertianShader archesUpperShader(
		&archesUpperTexture,
		archesUpperWidth,
		archesUpperHeight
	);

	TexturedLambertianShader outerRoofShader(
		&outerRoofTexture,
		outerRoofWidth,
		outerRoofHeight
	);

	TexturedLambertianShader outerPillarsShader(
		&outerPillarsTexture,
		outerPillarsWidth,
		outerPillarsHeight
	);

	TexturedLambertianShader halfPillarsShader(
		&halfPillarsTexture,
		halfPillarsWidth,
		halfPillarsHeight
	);

	TexturedLambertianShader windowFrameShader(
		&windowFrameTexture,
		windowFrameWidth,
		windowFrameHeight
	);

	TexturedLambertianShader flooringShader(
		&flooringTexture,
		flooringWidth,
		flooringHeight
	);

	TexturedLambertianShader roofDetailShader(
		&roofDetailTexture,
		roofDetailWidth,
		roofDetailHeight
	);

	TexturedLambertianShader wallDetailShader(
		&wallDetailTexture,
		wallDetailWidth,
		wallDetailHeight
	);

	TexturedLambertianShader pillarBaseShader(
		&pillarBaseTexture,
		pillarBaseWidth,
		pillarBaseHeight
	);

	TexturedLambertianShader innerPillarOneShader(
		&innerPillarOneTexture,
		innerPillarOneWidth,
		innerPillarOneHeight
	);

	TexturedLambertianShader innerPillarTwoShader(
		&innerPillarTwoTexture,
		innerPillarTwoWidth,
		innerPillarTwoHeight
	);

	TexturedLambertianShader altarBaseShader(
		&altarBaseTexture,
		altarBaseWidth,
		altarBaseHeight
	);

	TexturedLambertianShader altarBaseDetailShader(
		&altarBaseDetailTexture,
		altarBaseDetailWidth,
		altarBaseDetailHeight
	);

	TexturedLambertianShader altarTableShader(
		&altarTableTexture,
		altarTableWidth,
		altarTableHeight
	);

	TexturedLambertianShader windowDetailedShader(
		&windowDetailedTexture,
		windowDetailedWidth,
		windowDetailedHeight
	);

	TexturedLambertianShader windowBasicShader(
		&windowBasicTexture,
		windowBasicWidth,
		windowBasicHeight
	);

	TexturedLambertianShader windowUpperShader(
		&windowUpperTexture,
		windowUpperWidth,
		windowUpperHeight
	);

	TexturedLambertianShader seatsOneShader(
		&seatsOneTexture,
		seatsOneWidth,
		seatsOneHeight
	);

	TexturedLambertianShader seatsTwoShader(
		&seatsTwoTexture,
		seatsTwoWidth,
		seatsTwoHeight
	);

	TexturedLambertianShader seatsThreeShader(
		&seatsThreeTexture,
		seatsThreeWidth,
		seatsThreeHeight
	);

	MirrorShader mirrorShader;
	TexCoordTestShader texCoordTestShader;

	Scene scene;

	Model archesModel("../models/Arches.obj");
	Model archesInnerModel("../models/ArchesInner.obj");
	Model archesUpperModel("../models/ArchesUpper.obj");
	Model outerRoofModel("../models/OuterRoof.obj");
	Model outerPillarsModel("../models/OuterPillars.obj");
	Model halfPillarsModel("../models/HalfPillars.obj");
	Model windowFrameModel("../models/WindowFrame.obj");
	Model flooringModel("../models/Flooring.obj");
	Model roofDetailModel("../models/RoofDetail.obj");
	Model wallDetailModel("../models/WallDetail.obj");
	Model pillarBaseModel("../models/PillarBase.obj");
	Model innerPillarOneModel("../models/InnerPillar_one.obj");
	Model innerPillarTwoModel("../models/InnerPillar_two.obj");
	Model altarBaseModel("../models/AltarBase.obj");
	Model altarBaseDetailModel("../models/AltarBaseDetail.obj");
	Model altarTableModel("../models/AltarTable.obj");
	Model windowDetailedModel("../models/WindowDetailed.obj");
	Model windowBasicModel("../models/WindowBasic.obj");
	Model windowUpperModel("../models/WindowUpper.obj");
	Model seatsOneModel("../models/Seats_one.obj");
	Model seatsTwoModel("../models/Seats_two.obj");
	Model seatsThreeModel("../models/Seats_three.obj");

	Eigen::Vector3f sceneOffset(0.0f, -6.0f, 10.0f);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			archesModel,
			&archesShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			archesInnerModel,
			&archesInnerShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			archesUpperModel,
			&archesUpperShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			outerRoofModel,
			&outerRoofShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			outerPillarsModel,
			&outerPillarsShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			halfPillarsModel,
			&halfPillarsShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			windowFrameModel,
			&windowFrameShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			flooringModel,
			&flooringShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			roofDetailModel,
			&roofDetailShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			wallDetailModel,
			&wallDetailShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			pillarBaseModel,
			&pillarBaseShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			innerPillarOneModel,
			&innerPillarOneShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			innerPillarTwoModel,
			&innerPillarTwoShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			altarBaseModel,
			&altarBaseShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			altarBaseDetailModel,
			&altarBaseDetailShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			altarTableModel,
			&altarTableShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			windowDetailedModel,
			&windowDetailedShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			windowBasicModel,
			&windowBasicShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			windowUpperModel,
			&windowUpperShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			seatsOneModel,
			&seatsOneShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			seatsTwoModel,
			&seatsTwoShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	scene.renderables.push_back(
		std::make_shared<BVHNode>(
			seatsThreeModel,
			&seatsThreeShader,
			4,
			makeTranslationMatrix(sceneOffset)
		)
	);

	Eigen::Vector3f ambientLight(
		1.0f,
		0.9f,
		0.6f
	);

	std::vector<std::unique_ptr<Light>> lightSources;

	lightSources.push_back(
		std::make_unique<PointLight>(
			Eigen::Vector3f(-1.f, 3.f, -1.f),
			3.f * Eigen::Vector3f(1.f, 1.f, 1.f)
		)
	);

	lightSources.push_back(
		std::make_unique<DirectionalLight>(
			Eigen::Vector3f(0.f, -1.f, 1.f),
			0.5f * Eigen::Vector3f(1.f, 1.f, 1.f)
		)
	);

	lightSources.push_back(
		std::make_unique<DirectionalLight>(
			Eigen::Vector3f(-0.3f, 0.0f, -0.8f),
			0.8f * Eigen::Vector3f(1.0f, 0.95f, 0.8f)
		)
	);

	std::vector<unsigned int> scanlines(pixHeight);

	for (int i = 0; i < pixHeight; ++i)
	{
		scanlines[i] = i;
	}

	if (config["shuffleScanlines"])
	{
		std::random_device rd;
		std::mt19937 generator(rd());

		std::shuffle(
			scanlines.begin(),
			scanlines.end(),
			generator
		);
	}

	auto startTime = std::chrono::steady_clock::now();

#pragma omp parallel for
	for (int y = 0; y < pixHeight; ++y)
	{
		for (int x = 0; x < pixWidth; ++x)
		{
			Ray ray = cam.getRay(x, scanlines[y]);

			HitInfo hitInfo;

			int line =
				(pixHeight - scanlines[y]) - 1;

			if (scene.intersect(
				ray,
				1e-6f,
				FLT_MAX,
				hitInfo,
				VISIBLE_BITMASK))
			{
				Eigen::Vector3f color =
					hitInfo.shader->getColor(
						hitInfo,
						&scene,
						lightSources,
						ambientLight,
						0,
						config["maxBounces"]
					);

				color.x() = std::min(color.x(), 1.f);
				color.y() = std::min(color.y(), 1.f);
				color.z() = std::min(color.z(), 1.f);

				outImage[(x + line * pixWidth) * nChannels + 0] =
					color.x() * 255;

				outImage[(x + line * pixWidth) * nChannels + 1] =
					color.y() * 255;

				outImage[(x + line * pixWidth) * nChannels + 2] =
					color.z() * 255;

				outImage[(x + line * pixWidth) * nChannels + 3] =
					255;
			}
			else
			{
				outImage[(x + line * pixWidth) * nChannels + 0] = 0;
				outImage[(x + line * pixWidth) * nChannels + 1] = 0;
				outImage[(x + line * pixWidth) * nChannels + 2] = 0;
				outImage[(x + line * pixWidth) * nChannels + 3] = 255;
			}
		}

		if (omp_get_thread_num() ==
			omp_get_num_threads() - 1)
		{
			std::clog
				<< "\rScanlines remaining: "
				<< (pixHeight - y)
				<< ' '
				<< std::flush;
		}
	}

	auto renderTime =
		std::chrono::steady_clock::now()
		- startTime;

	std::cout
		<< "Render duration "
		<< std::chrono::duration_cast<
		std::chrono::milliseconds>(
			renderTime
		).count() * 1e-3f
		<< " seconds."
		<< std::endl;

	lodepng::encode(
		config["outputFilename"],
		outImage,
		pixWidth,
		pixHeight
	);

	Ray ray = cam.getRay(531, 325);

	HitInfo hitInfo;

	if (scene.intersect(
		ray,
		1e-6f,
		FLT_MAX,
		hitInfo,
		VISIBLE_BITMASK))
	{
		std::cerr << "HIT!\n";
	}

	return 0;
}