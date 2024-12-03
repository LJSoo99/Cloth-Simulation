#define JGL2_IMPLEMENTATION
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <JGL2/JGL.hpp>
#include <JGL2/Simulation3DView.hpp>

using namespace JGL2;
using namespace jm;

float alpha = .1;
float ks = 1500;

Simulation3DView<JR::PBRRenderer>* view = nullptr;

struct particle {
	vec3 x;
	vec3 v = vec3(0);
	vec3 f;
	float m = 1;
	bool pinned = false;

	particle(const vec3& p, float mass = 1) :
		x(p), m(mass) {}
	void clearForce() { f = vec3(0); }
	void addForce(const vec3& ff) { f += ff; }
	void integrate(float dt) {
		if (pinned) {
			v = vec3(0);
			return;
		}
		v = v + f / m * dt;
		x = x + v * dt;
	}
	void render() const {
		JR::drawSphere(x, 1, vec4(1, .6, 0, 1));
	}
};

struct spring {
	float kd = 1.0;
	float r;
	particle& a;
	particle& b;
	spring(particle& p1, particle& p2) :
		a(p1), b(p2), r(length(p2.x - p1.x)) {}
	void addForce() {
		vec3 dX = b.x - a.x;
		vec3 nX = normalize(dX);
		vec3  f = -ks * (length(dX) - r) * nX;
		f += kd * dot(a.v - b.v, nX) * nX;
		a.addForce(-f);
		b.addForce(f);
	}
	void render() const {
		JR::drawCylinder(a.x, b.x, 0.6, vec4(0, 1, 0, 1));
	}
};

struct collider {
	virtual void resolveCollision(particle&) = 0;
};

struct plane : collider {
	vec3 x;
	vec3 N;
	plane(vec3 p0, vec3 n0) :x(p0), N(n0) {}
	void render() const {
		JR::drawQuad(x, N, jm::vec2(1000), jm::vec4(0, 0, .4, 1));
	}
	virtual void resolveCollision(particle& p) {
		if (dot(p.x - x, N) < 0.1 && dot(p.v, N) < 0) {
			vec3 vn = dot(p.v, N) * N;
			vec3 vt = p.v - vn;
			p.v = vt - alpha * vn;
//			p.x = (p.x - x) - dot(p.x - x, N) * N + x;
		}
		if (dot(p.x - x, N) < 0.1 && abs(dot(p.v, N)) < 0.1) {
			vec3 vn = dot(p.v, N) * N;
			vec3 vt = p.v - vn;
			p.v = vt;
			p.x = p.x - dot(p.x - x, N) * N;
		}
	}
};

struct sphere : collider {
	vec3 c;
	vec3 n;
	float r;
	sphere(vec3 c0, float r0) : c(c0), r(r0) {}
	void render() const {
		JR::drawSphere(c, r, vec4(1, 0, 1, 0.85));
	}
	virtual void resolveCollision(particle& p) {
		vec3 d = p.x - c;

		if (length(d) < r) {

			n = normalize(d);
			float u = r - length(d);
			float v = dot(p.v, n);
			float k = 0.01;

			if (v < -k) {
				vec3 vn = v * n;
				vec3 vt = p.v - vn;
				p.v = vt - alpha * vn;
			}

			else if (v < k) {
				vec3 vn = v * n;
				vec3 vt = p.v - vn;
				p.v = vt;
			}

			p.x += u * n;
		}

	}
};

std::vector<particle> particles;
std::vector<spring> springs;
plane thePlane(vec3(0), vec3(0, 1, 0));
sphere theSphere(vec3(-2.5, 27, 0.1), 20);


const vec3 G = vec3(0, -980, 0);
float kd = 1;
int n = 100;

const int rows = 10;
const int cols = 10;

float frand() {
	return rand() / float(RAND_MAX);
}

void init() {
	const int rows = 10;
	const int cols = 10;
	particles.clear();
	springs.clear();
	for (int y = 0; y < rows; y++) {
		for (int x = 0; x < cols; x++) {
			particles.emplace_back(jm::vec3((x - cols / 2) / float(cols) * 50 + frand() * 0.1, (y - rows / 2) / float(rows) * 50 + 75 + frand() * 0.1, frand() * 0.1));
		}

	}
	for (int y = 0; y < rows; y++) {
		for (int x = 0; x < cols - 1; x++) {
			springs.emplace_back(particles[x + y * cols], particles[x + 1 + y * cols]);
		}
	}
	for (int y = 0; y < rows - 1; y++) {
		for (int x = 0; x < cols; x++) {
			springs.emplace_back(particles[x + y * cols], particles[x + (y + 1) * cols]);
		}
	}
	for (int y = 0; y < rows - 1; y++) {
		for (int x = 0; x < cols - 1; x++) {
			springs.emplace_back(particles[x + y * cols], particles[x + 1 + (y + 1) * cols]);
		}
	}
	for (int y = 0; y < rows - 1; y++) {
		for (int x = 0; x < cols - 1; x++) {
			springs.emplace_back(particles[x + 1 + y * cols], particles[x + (y + 1) * cols]);
		}
	}
	particles[(rows - 1) * cols].pinned = true;
	particles[rows * cols - 1].pinned = true;

//	particles.emplace_back(jm::vec3( 0, 100, 0 ));
//	particles.emplace_back(jm::vec3( 30, 100, 0 ));
//	particles[0].pinned = true;
//	springs.emplace_back(particles[0], particles[1]);
}

void render() {
	for (auto& p : particles)
		p.render();
	for (auto& s : springs)
		s.render();
	thePlane.render();
	theSphere.render();

}

bool move3D(const vec3& p) {
	return false;
}

bool drag3D(const vec3& delta) {
	return false;
}

void frame(float dt) {
	for (int iter = 0; iter < n; iter++) {
		for (auto& p : particles)
			p.clearForce();

		//Add forces
		for (auto& p : particles)
			p.addForce(p.m * G);

		for (auto& p : particles)
			p.addForce(-kd * p.v);

		for (auto& s : springs)
			s.addForce();

		for (auto& p : particles)
			p.integrate(dt / n);

		for (auto& p : particles)
			thePlane.resolveCollision(p);

		for (auto& p : particles)
			theSphere.resolveCollision(p);
	}
}

bool keyCB(int k) {
	if (k == '1') {
		particles[cols * (rows - 1)].pinned = !particles[cols * (rows - 1)].pinned;
		return true;
	}
	if (k == '2') {
		particles[cols * rows - 1].pinned = !particles[cols * rows - 1].pinned;
		return true;
	}
	return false;
}

int main() {
	Window* win = new Window(800, 600, "cloth simulation");
	win->alignment(align_t::ALL);
	view = new Simulation3DView<JR::PBRRenderer>(0, 0, 800, 600, "View");
	view->move3DCB(move3D);
	view->initCB(init);
	view->drag3DCB(drag3D);
	view->frameCB(frame);
	view->renderFunc(render);
	view->keyCB(keyCB);
	view->addQuickUI(new nanoSlider<float>(0, 0, 200, "kd", 0, 1, kd));
	view->addQuickUI(new nanoSlider<float>(0, 0, 200, "ks", 0, 1000, ks));
	win->show();
	_JGL::run();
	return 0;
}