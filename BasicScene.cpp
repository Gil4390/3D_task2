#include "BasicScene.h"
#include "IglMeshLoader.h"
#include <queue>
#include "per_vertex_normals.h"

#define Pair std::pair<igl::AABB< Eigen::MatrixXd, 3>, igl::AABB< Eigen::MatrixXd, 3>>

using namespace cg3d;

std::queue<Pair> queue;
//int SCALE_FACTOR = 16;

void BasicScene::Init(float fov, int width, int height, float near, float far)
{
	camera = Camera::Create("camera", fov, float(width) / float(height), near, far);
	AddChild(root = Movable::Create("root")); // a common (invisible) parent object for all the shapes
	auto daylight{ std::make_shared<Material>("daylight", "shaders/cubemapShader") };
	daylight->AddTexture(0, "textures/cubemaps/Daylight Box_", 3);
	auto background{ Model::Create("background", Mesh::Cube(), daylight) };
	AddChild(background);
	background->Scale(120, Axis::XYZ);
	background->SetPickable(false);
	background->SetStatic();

	auto program = std::make_shared<Program>("shaders/basicShader");
	auto material = std::make_shared<Material>("material", program); // empty material
	material->AddTexture(0, "textures/box0.bmp", 2);
	auto material2 = std::make_shared<Material>("material", program); // empty material
	material2->AddTexture(0, "textures/grass.bmp", 2);
	// defines 2 meshes of cows
	auto obj1Mesh{ IglLoader::MeshFromFiles("obj1_igl", "data/cow.off") };
	auto obj2Mesh{ IglLoader::MeshFromFiles("obj2_igl", "data/cow.off") };
	// create 2 models fron the cow meshes
	obj1 = Model::Create("bunny1", obj1Mesh, material);
	obj2 = Model::Create("bunny2", obj2Mesh, material);

	camera->Translate(2, Axis::Z);
	root->AddChild(obj1);
	root->AddChild(obj2);
	// initializing the AABB trees of the aligned boxes(happens just once)
	tree1.init(obj1->GetMeshList()[0]->data[0].vertices, obj1->GetMeshList()[0]->data[0].faces);
	tree2.init(obj2->GetMeshList()[0]->data[0].vertices, obj2->GetMeshList()[0]->data[0].faces);
	// creating the box that will surrond the first object
	auto cube1{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
	bb1 = Model::Create("bb1", cube1, material2);
	bb1->showFaces = false;
	bb1->showWireframe = true;
	obj1->AddChild(bb1);
	// creating the box that will surrond the second object
	auto cube2{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
	bb2 = Model::Create("bb2", cube2, material2);
	bb2->showFaces = false;
	bb2->showWireframe = true;
	obj2->AddChild(bb2);
	// creating the box that will show the place that was detect of the first object
	auto cube3{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
	fbb1 = Model::Create("fbb1", cube3, material2);
	// creating the box that will show the place that was detect of the second object
	auto cube4{ IglLoader::MeshFromFiles("cube_igl","data/cube.off") };
	fbb2 = Model::Create("fbb2", cube4, material2);
	// function for display the bounding boxes around the objects
	CreateBox(bb1, tree1.m_box);
	CreateBox(bb2, tree2.m_box);
	// translate the objects so they dont intersects in the beggining
	obj1->Translate({ -0.6,0,0 });
	obj1->showWireframe = true;
	obj2->Translate({ 0.6,0,0 });
	obj2->showWireframe = true;

	dx = 0;
	dy = 0;
}

// function for creating a mesh from vertices and faces
void BasicScene::CreateBox(std::shared_ptr<cg3d::Model> model, Eigen::AlignedBox3d box)
{
	Eigen::Vector3d m1 = box.max();
	Eigen::Vector3d M1 = box.min();

	// Corners of the bounding box
	Eigen::MatrixXd V_box1(8, 3);
	V_box1 <<
		m1(0), m1(1), m1(2),
		M1(0), m1(1), m1(2),
		M1(0), M1(1), m1(2),
		m1(0), M1(1), m1(2),
		m1(0), m1(1), M1(2),
		M1(0), m1(1), M1(2),
		M1(0), M1(1), M1(2),
		m1(0), M1(1), M1(2);

	//Faces of the bounding box
	Eigen::MatrixXi F_box1(12, 3);
	F_box1 <<
		0, 1, 3,
		1, 2, 3,
		0, 3, 4,
		3, 4, 7,
		4, 5, 7,
		5, 6, 7,
		1, 2, 6,
		1, 5, 6,
		3, 6, 7,
		2, 3, 6,
		0, 1, 4,
		1, 4, 5;
	Eigen::MatrixXd N1, T1;
	igl::per_vertex_normals(V_box1, F_box1, N1);
	T1 = Eigen::MatrixXd::Zero(V_box1.rows(), 2);
	// updated the mesh list of the model
	std::vector<std::shared_ptr<Mesh>> meshList1 = model->GetMeshList();
	meshList1[0]->data.push_back({ V_box1, F_box1, N1, T1 });
	model->SetMeshList(meshList1);
	model->meshIndex += 1;
}

void BasicScene::Update(const Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model)
{
	Scene::Update(program, proj, view, model);
	program.SetUniform4f("lightColor", 1.0f, 1.0f, 1.0f, 0.5f);
	program.SetUniform4f("Kai", 1.0f, 1.0f, 1.0f, 1.0f);

	// check if the object needs to move or maybe happend a collision and he should not
	if (shouldMove) {
		obj1->Translate(dx, Axis::X);
		obj1->Translate(dy, Axis::Y);
		// checking a detection
		if (CheckCollisionDetection()) {
			dx = 0;
			dy = 0;
			shouldMove = false;
			printf("collision detected\n");
			//to make the boxes apear in the correct spot
			fbb1->RotateByDegree(360, Axis::X);
			fbb2->RotateByDegree(360, Axis::X);
		}
	}
}

bool BasicScene::CheckCollisionDetection() {
	// Initialize with main bounding box for each object
	queue.push(Pair(tree1, tree2));
	// Now, we need to Repeatedly pull next potential pair off queue and test for intersection
	while (!queue.empty()) {
		// taking the first element from the queue
		Pair firstInQueue = queue.front();
		queue.pop();
		// we need to check if there is an intersection between the boxes
		if (Intersects(firstInQueue.first, firstInQueue.second))
		{
			if (firstInQueue.first.is_leaf() && firstInQueue.second.is_leaf())
			{
				// there is a collision so we need to draw the small boxes around the place of the detection
				//===============first box============
				CreateBox(fbb1, firstInQueue.first.m_box);
				fbb1->showFaces = true;
				fbb1->showWireframe = true;
				obj1->AddChild(fbb1);

				//===============second box============
				CreateBox(fbb2, firstInQueue.second.m_box);
				fbb2->showFaces = true;
				fbb2->showWireframe = true;
				obj2->AddChild(fbb2);

				return true;
			}
			else if (firstInQueue.first.is_leaf() && !firstInQueue.second.is_leaf())
			{
				queue.push(Pair(firstInQueue.first, *firstInQueue.second.m_left));
				queue.push(Pair(firstInQueue.first, *firstInQueue.second.m_right));
			}
			else if (!firstInQueue.first.is_leaf() && firstInQueue.second.is_leaf())
			{
				queue.push(Pair(*firstInQueue.first.m_left, firstInQueue.second));
				queue.push(Pair(*firstInQueue.first.m_right, firstInQueue.second));
			}
			else
			{
				queue.push(Pair(*firstInQueue.first.m_left, *firstInQueue.second.m_left));
				queue.push(Pair(*firstInQueue.first.m_left, *firstInQueue.second.m_right));
				queue.push(Pair(*firstInQueue.first.m_right, *firstInQueue.second.m_left));
				queue.push(Pair(*firstInQueue.first.m_right, *firstInQueue.second.m_right));
			}
		}
	}
	return false;
}

bool BasicScene::Intersects(igl::AABB< Eigen::MatrixXd, 3> t1, igl::AABB< Eigen::MatrixXd, 3> t2) {
	// extract and define all the variables we need for checking the 15 terms
	Eigen::Vector3d D = MakeDvector(t1, t2);

	Eigen::Vector3d A0, A1, A2;
	A0 = obj1->GetRotation().cast<double>().col(0);
	A1 = obj1->GetRotation().cast<double>().col(1);
	A2 = obj1->GetRotation().cast<double>().col(2);

	float a0, a1, a2;
	a0 = t1.m_box.sizes()(0) / 2;
	a1 = t1.m_box.sizes()(1) / 2;
	a2 = t1.m_box.sizes()(2) / 2;

	Eigen::Vector3d B0, B1, B2;
	B0 = obj2->GetRotation().cast<double>().col(0);
	B1 = obj2->GetRotation().cast<double>().col(1);
	B2 = obj2->GetRotation().cast<double>().col(2);

	float b0, b1, b2;
	b0 = t2.m_box.sizes()(0) / 2;
	b1 = t2.m_box.sizes()(1) / 2;
	b2 = t2.m_box.sizes()(2) / 2;

	Eigen::Matrix3d M = obj1->GetRotation().cast<double>().transpose() * obj2->GetRotation().cast<double>();

	float c00, c01, c02,
		c10, c11, c12,
		c20, c21, c22;
	c00 = M.row(0)(0);
	c01 = M.row(0)(1);
	c02 = M.row(0)(2);
	c10 = M.row(1)(0);
	c11 = M.row(1)(1);
	c12 = M.row(1)(2);
	c20 = M.row(2)(0);
	c21 = M.row(2)(1);
	c22 = M.row(2)(2);

	// check all the 15 terms

	// case 1
	auto res1 = abs(A0.transpose() * D);
	if (res1 > a0 + b0 * abs(c00) + b1 * abs(c01) + b2 * abs(c02))
		return false;

	// case 2
	auto res2 = abs(A1.transpose() * D);
	if (res2 > a1 + b0 * abs(c10) + b1 * abs(c11) + b2 * abs(c12))
		return false;

	// case 3
	auto res3 = abs(A2.transpose() * D);
	if (res3 > a2 + b0 * abs(c20) + b1 * abs(c21) + b2 * abs(c22))
		return false;

	// case 4
	auto res4 = abs(B0.transpose() * D);
	if (res4 > b0 + a0 * abs(c00) + a1 * abs(c10) + a2 * abs(c20))
		return false;

	// case 5
	auto res5 = abs(B1.transpose() * D);
	if (res5 > b1 + a0 * abs(c01) + a1 * abs(c11) + a2 * abs(c21))
		return false;

	// case 6
	auto res6 = abs(B2.transpose() * D);
	if (res6 > b2 + a0 * abs(c02) + a1 * abs(c12) + a2 * abs(c22))
		return false;

	// case 7
	float R7 = (c10 * A2.transpose() * D);
	float RR7 = (c20 * A1.transpose() * D);
	float res7 = abs(R7 - RR7);
	if (res7 > a1 * abs(c20) + a2 * abs(c10) + b1 * abs(c02) + b2 * abs(c01))
		return false;

	// case 8
	float R8 = (c11 * A2.transpose() * D);
	float RR8 = (c21 * A1.transpose() * D);
	float res8 = abs(R8 - RR8);
	if (res8 > a1 * abs(c21) + a2 * abs(c11) + b0 * abs(c02) + b2 * abs(c00))
		return false;

	// case 9
	float R9 = (c12 * A2.transpose() * D);
	float RR9 = (c22 * A1.transpose() * D);
	float res9 = abs(R9 - RR9);
	if (res9 > a1 * abs(c22) + a2 * abs(c12) + b0 * abs(c01) + b1 * abs(c00))
		return false;

	// case 10
	float R10 = (c20 * A0.transpose() * D);
	float RR10 = (c00 * A2.transpose() * D);
	float res10 = abs(R10 - RR10);
	if (res10 > a0 * abs(c20) + a2 * abs(c00) + b1 * abs(c12) + b2 * abs(c11))
		return false;

	// case 11
	float R11 = (c21 * A0.transpose() * D);
	float RR11 = (c01 * A2.transpose() * D);
	float res11 = abs(R11 - RR11);
	if (res11 > a0 * abs(c21) + a2 * abs(c01) + b0 * abs(c12) + b2 * abs(c10))
		return false;

	// case 12
	float R12 = (c22 * A0.transpose() * D);
	float RR12 = (c02 * A2.transpose() * D);
	float res12 = abs(R12 - RR12);
	if (res12 > a0 * abs(c22) + a2 * abs(c02) + b0 * abs(c11) + b1 * abs(c10))
		return false;

	// case 13
	float R13 = (c00 * A1.transpose() * D);
	float RR13 = (c10 * A0.transpose() * D);
	float res13 = abs(R13 - RR13);
	if (res13 > a0 * abs(c10) + a1 * abs(c00) + b1 * abs(c22) + b2 * abs(c21))
		return false;

	// case 14
	float R14 = (c01 * A1.transpose() * D);
	float RR14 = (c11 * A0.transpose() * D);
	float res14 = abs(R14 - RR14);
	if (res14 > a0 * abs(c11) + a1 * abs(c01) + b0 * abs(c22) + b2 * abs(c20))
		return false;

	// case 15
	float R15 = (c02 * A1.transpose() * D);
	float RR15 = (c12 * A0.transpose() * D);
	float res15 = abs(R15 - RR15);
	if (res15 > a0 * abs(c12) + a1 * abs(c02) + b0 * abs(c21) + b1 * abs(c20))
		return false;

	return true;
}

Eigen::Vector3d BasicScene::MakeDvector(igl::AABB< Eigen::MatrixXd, 3> t1, igl::AABB< Eigen::MatrixXd, 3> t2)
{
	Eigen::Vector4d C1, C2;
	auto center1 = t1.m_box.center();
	Eigen::Vector4d cent1 = { center1[0], center1[1], center1[2], 1 };
	C1 = obj1->GetTransform().cast<double>() * cent1;

	auto center2 = t2.m_box.center();
	Eigen::Vector4d cent2 = { center2[0], center2[1], center2[2], 1 };
	C2 = obj2->GetTransform().cast<double>() * cent2;

	return (C2 - C1).head(3);
}

void BasicScene::KeyCallback(Viewport* viewport, int x, int y, int key, int scancode, int action, int mods) {
	auto system = camera->GetRotation().transpose();

	if (action == GLFW_PRESS || action == GLFW_REPEAT) {
		switch (key) // NOLINT(hicpp-multiway-paths-covered)
		{
		case GLFW_KEY_ESCAPE:
			glfwSetWindowShouldClose(window, GLFW_TRUE);
			break;
		case GLFW_KEY_UP:
			//camera->RotateInSystem(system, 0.1f, Axis::X);
			shouldMove = true;
			dx = 0;
			dy = 0.005;
			break;
		case GLFW_KEY_DOWN:
			//camera->RotateInSystem(system, -0.1f, Axis::X);
			shouldMove = true;
			dx = 0;
			dy = -0.005;
			break;
		case GLFW_KEY_LEFT:
			//camera->RotateInSystem(system, 0.1f, Axis::Y);
			shouldMove = true;
			dx = -0.005;
			dy = 0;
			break;
		case GLFW_KEY_RIGHT:
			//camera->RotateInSystem(system, -0.1f, Axis::Y);
			shouldMove = true;
			dx = 0.005;
			dy = 0;
			break;
		case GLFW_KEY_W:
			camera->TranslateInSystem(system, { 0, 0.05f, 0 });
			break;
		case GLFW_KEY_S:
			camera->TranslateInSystem(system, { 0, -0.05f, 0 });
			break;
		case GLFW_KEY_A:
			camera->TranslateInSystem(system, { -0.05f, 0, 0 });
			break;
		case GLFW_KEY_D:
			camera->TranslateInSystem(system, { 0.05f, 0, 0 });
			break;
		case GLFW_KEY_B:
			camera->TranslateInSystem(system, { 0, 0, 0.05f });
			break;
		case GLFW_KEY_F:
			camera->TranslateInSystem(system, { 0, 0, -0.05f });
			break;
		case GLFW_KEY_SPACE:
			shouldMove = false;
			dx = 0;
			dy = 0;
			break;
		}
	}
}

