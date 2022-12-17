#pragma once

#include "Scene.h"
#include "AABB.h"
#include "AutoMorphingModel.h"
#include <utility>

class BasicScene : public cg3d::Scene
{
public:
    explicit BasicScene(std::string name, cg3d::Display* display) : Scene(std::move(name), display) {};
    void Init(float fov, int width, int height, float near, float far);
    void CreateBox(std::shared_ptr<cg3d::Model> model, Eigen::AlignedBox3d box);
    void Update(const cg3d::Program& program, const Eigen::Matrix4f& proj, const Eigen::Matrix4f& view, const Eigen::Matrix4f& model) override;

    bool CheckCollisionDetection();
    bool Intersects(igl::AABB<Eigen::MatrixXd, 3> t1, igl::AABB<Eigen::MatrixXd, 3> t2);
    Eigen::Vector3d MakeDvector(igl::AABB<Eigen::MatrixXd, 3> t1, igl::AABB<Eigen::MatrixXd, 3> t2);
    void KeyCallback(cg3d::Viewport* viewport, int x, int y, int key, int scancode, int action, int mods) override;

private:
    bool shouldMove = false;
    float dx, dy;
    std::shared_ptr<Movable> root;
    std::shared_ptr<cg3d::Model> obj1, obj2;
    std::shared_ptr<cg3d::Model> bb1, bb2, fbb1, fbb2;
    igl::AABB< Eigen::MatrixXd, 3> tree1, tree2;
};
