//
// Created by jie on 9/7/18.
//

#include "log.hpp"
#include "bogus_integrator.h"
#include "SolverOptions.hh"
#include "ClothFrictionSolver.hh"
#include "ClothFrictionData.hh"

#include <bogus/Core/Block.io.hpp>
#include <bogus/Core/Block.impl.hpp>

double A_scale = 1;
double b_scale = 1;
double eval_scale = 1;

void initial_stiff_matrix(const arcsim::SpMat<arcsim::Mat3x3> &A, argus::ClothFrictionData::StiffnessMatrixType &M) {
    int m = A.m, n = A.n;
    M.setRows(m);
    M.setCols(n);
    for (int i = 0; i < m; i++) {
        const arcsim::SpVec<arcsim::Mat3x3> &row = A.rows[i];
        size_t *indices = new size_t[n];
        for (int j = 0; j < n; j++) {
            indices[j] = -1;
        }
        for (size_t jj = 0; jj < row.indices.size(); jj++) {
            indices[row.indices[jj]] = jj;
        }
        for (int j = 0; j < n; j++) {
            if (indices[j] == -1) {
                continue;
            }
            M.insertBack(i, j);
        }
        delete[] indices;
    }
    M.finalize();
    for (int i = 0; i < m; i++) {
        const arcsim::SpVec<arcsim::Mat3x3> &row = A.rows[i];
        for (size_t jj = 0; jj < row.indices.size(); jj++) {
            int j = row.indices[jj];
            arcsim::Mat3x3 entry = A_scale*row.entries[jj];
            M.block(i, j).row(0)<<entry(0, 0), entry(0, 1), entry(0, 2);
            M.block(i, j).row(1)<<entry(1, 0), entry(1, 1), entry(1, 2);
            M.block(i, j).row(2)<<entry(2, 0), entry(2, 1), entry(2, 2);
        }
    }
}

void reverse_stiff_matrix(const argus::ClothFrictionData::StiffnessMatrixType &M, arcsim::SpMat<arcsim::Mat3x3> &A) {
    int m = M.rowsOfBlocks();
    int n = M.colsOfBlocks();
    A.m = m;
    A.n = n;
    A.rows.resize(m);
    for (int i = 0; i < m; i++) {
        for (argus::ClothFrictionData::StiffnessMatrixType::InnerIterator it (M.innerIterator(i)); it ; ++ it) {
            int col = it.inner();
            argus::Mat3 b = M.block(it.ptr());
            arcsim::Mat3x3 block;
            for (int k = 0; k < 9; k++) {
                block(k/3, k%3) = b(k/3, k%3);
            }
            A(i, col) = block;
        }
    }
}

void initial_f(const vector<arcsim::Vec3> &b, argus::DynVec &f) {
    f.resize(3*b.size());
    for (int i = 0; i < b.size(); i++) {
        f.segment(i*3, 3)<<b[i][0], b[i][1], b[i][2];
    }
    f *= b_scale;
}

void reverse_f(const argus::DynVec &f, vector<arcsim::Vec3> &b) {
    b.resize(f.size()/3);
    for (int i = 0; i < b.size(); i++) {
        arcsim::Vec3 fi(f[i*3], f[i*3 + 1], f[i*3 + 2]);
        b[i] = fi;
    }
}

bool find_in_mesh(const arcsim::Node* node, const arcsim::Mesh &mesh) {
    for (int n = 0; n < mesh.nodes.size(); n++) {
        if (node == mesh.nodes[n]) {
            return true;
        }
    }
    return false;
}

void initial_contact_info(const vector<arcsim::Impact> impacts, arcsim::Mesh &mesh, argus::DynVec &mu,
                          argus::ClothFrictionData::ContactBasisMatrix &cb, argus::DynMat3i &iA, argus::DynMat3 &baryA,
                          argus::DynMat3i &iB, argus::DynMat3 &baryB, argus::DynMat3 &affineVel, const double dt) {
    int contact_number = impacts.size();
    mu.resize(contact_number);
    cb.setRows(contact_number);
    cb.setCols(contact_number);
    iA.resize(3, contact_number);
    baryA.resize(3, contact_number);
    iB.resize(3, contact_number);
    baryB.resize(3, contact_number);
    affineVel.resize(3, contact_number);

    for (int i = 0; i < contact_number; i++) {
        cb.insertBack(i, i);
    }
    cb.finalize();

    for (int i = 0; i < contact_number; i++) {
        const arcsim::Impact &impact = impacts[i];

        mu[i] = arcsim::magic.friction_coeff;

        arcsim::Vec3 xA;
        arcsim::Vec3 xB;
        arcsim::Vec3 vA;
        arcsim::Vec3 vB;
        arcsim::Vec3 normal = normalize(impact.n);
        if (impact.type == arcsim::Impact::VF) {
            if (impact.inverted) {
                iA.col(i) << impact.nodes[1]->index, impact.nodes[2]->index, impact.nodes[3]->index;
                baryA.col(i) << -impact.w[1], -impact.w[2], -impact.w[3];
                iB.col(i) << -1, -1, -1;
                baryB.col(i) << 0, 0, 0;
                xA = arcsim::Vec3(0, 0, 0);
                for (int n = 1; n <= 3; n++) {
                    xA += impact.nodes[n]->x*(-impact.w[n]);
                }
                vA = arcsim::Vec3(0, 0, 0);
                xB = impact.nodes[0]->x;
                vB = impact.nodes[0]->v;

                normal = -normal;
            } else if (impact.self) {
                iA.col(i) << impact.nodes[0]->index, -1, -1;
                baryA.col(i) << impact.w[0], 0, 0;
                iB.col(i) << impact.nodes[1]->index, impact.nodes[2]->index, impact.nodes[3]->index;
                baryB.col(i) << impact.w[1], impact.w[2], impact.w[3];
                xA = impact.nodes[0]->x;
                vA = arcsim::Vec3(0);
                xB = arcsim::Vec3(0, 0, 0);
                for (int n = 1; n <= 3; n++) {
                    xB += impact.nodes[n]->x*(-impact.w[n]);
                }
                vB = arcsim::Vec3(0, 0, 0);
            } else {
                iA.col(i) << impact.nodes[0]->index, -1, -1;
                baryA.col(i) << impact.w[0], 0, 0;
                iB.col(i) << -1, -1, -1;
                baryB.col(i) << 0, 0, 0;
                xA = impact.nodes[0]->x;
                vA = arcsim::Vec3(0);
                xB = arcsim::Vec3(0, 0, 0);
                for (int n = 1; n <= 3; n++) {
                    xB += impact.nodes[n]->x*(-impact.w[n]);
                }
                vB = arcsim::Vec3(0, 0, 0);
                for (int n = 1; n <= 3; n++) {
                    vB += impact.nodes[n]->v*(-impact.w[n]);
                }
            }
        } else if (impact.type == arcsim::Impact::EE) {

            iA.col(i) << impact.nodes[0]->index, impact.nodes[1]->index, -1;
            baryA.col(i) << impact.w[0], impact.w[1], 0;
            xA = impact.nodes[0]->x*impact.w[0] + impact.nodes[1]->x*impact.w[1];
            vA = arcsim::Vec3(0, 0, 0);
            if (impact.self) {
                iB.col(i) << impact.nodes[2]->index, impact.nodes[3]->index, -1;
                baryB.col(i) << impact.w[2], impact.w[3], 0;
                xB = impact.nodes[2]->x*(-impact.w[2]) + impact.nodes[3]->x*(-impact.w[3]);
                vB = arcsim::Vec3(0, 0, 0);
            } else {
                iB.col(i) << -1, -1, -1;
                baryB.col(i) << 0, 0, 0;
                xB = impact.nodes[2]->x*(-impact.w[2]) + impact.nodes[3]->x*(-impact.w[3]);
                vB = impact.nodes[2]->v*(-impact.w[2]) + impact.nodes[3]->v*(-impact.w[3]);
            }
        }

        arcsim::Vec3 delX = xA - xB;
        arcsim::Vec3 delV = vA - vB;
        arcsim::Vec3 con = delV + delX/dt - arcsim::magic.projection_thickness*normal/dt;  // todo change this
        affineVel.col(i)<<con[0], con[1], con[2];

        arcsim::Vec3 t1, t2;

        if( fabs( dot(normal,arcsim::Vec3(1,0,0)) ) < 0.1 || fabs( dot(normal,arcsim::Vec3(1,0,0)) ) > 0.9 ){
            t1 = cross(normal, arcsim::Vec3(0, 1, 0));
            t2 = cross(normal, t1);
        } else {
            t1 = cross(normal, arcsim::Vec3(1, 0, 0));
            t2 = cross(normal, t1);
        }
        normal = normalize(normal);

        t1 = normalize(t1);
        t2 = normalize(t2);

        cb.block(i, i).col(0)<<normal[0], normal[1], normal[2];
        cb.block(i, i).col(1)<<t1[0], t1[1], t1[2];
        cb.block(i, i).col(2)<<t2[0], t2[1], t2[2];

        // 	pair<arcsim::Vec3, arcsim::Vec3> v_and_x1, v_and_x2;

        // 	iA.col(i) << impact.nodeA -> index , impact.nodeA->index , impact.nodeA->index;  // normally the last two would be like -1
        // 	baryA.col(i) << 1 , 0 , 0;
        // 	xA = impact.nodeA->x;
        // 	vA = impact.nodeA->v;

        // 	// Self-contact case
        // 	if( impact.nodeB != 0 ){
        // 		std::cout << "self contact case \n";
        // 		iB.col(i) << impact.nodeB -> index , impact.nodeB -> index, impact.nodeB -> index;
        // 		baryB.col(i) << 1 , 0 , 0;
        // 		xB = impact.nodeB->x;
        // 		// vB = impact.nodeB->v;
        // 		vB = arcsim::Vec3(0);
        // 	// Not self-contact (collision is with an external object)
        // 	} else {


        // 		iB.col(i) << -1 , -1 , -1;
        // 		baryB.col(i) << 0 , 0 , 0;
        // 		xB = impact.posB;
        // 		vB = impact.velB;

        // 	}


        // 	arcsim::Vec3 normal = normalize(impact.normal);
        // 	arcsim::Vec3 vCloth = arcsim::Vec3(0);
        // 	arcsim::Vec3 vObs = vB;
        // 	arcsim::Vec3 xCloth = xA;
        // 	arcsim::Vec3 xObs = xB;

        // 	// arcsim::Vec3 delX = dot( (xCloth - xObs) , normal )*normal;
        // 	arcsim::Vec3 delX = xCloth - xObs;

        // 	// arcsim::Vec3 delV = dot(vCloth - vObs, normal)*normal;
        // 	arcsim::Vec3 delV = vCloth - vObs;
        // 	arcsim::Vec3 con = delV + delX/dt - .25*::magic.projection_thickness*normal/dt;  // todo change this
        // 	affineVel.col(i)<<con[0], con[1], con[2];

        // 	arcsim::Vec3 t1, t2;

        // 	if( fabs( dot(normal,arcsim::Vec3(1,0,0)) ) < 0.1 || fabs( dot(normal,arcsim::Vec3(1,0,0)) ) > 0.9 ){
        // 		t1 = cross(normal, arcsim::Vec3(0, 1, 0));
        // 		t2 = cross(normal, t1);
        // 	} else {
        // 		t1 = cross(normal, arcsim::Vec3(1, 0, 0));
        // 		t2 = cross(normal, t1);
        // 	}
        // 	normal = normalize(normal);

        // 	t1 = normalize(t1);
        // 	t2 = normalize(t2);

        // 	cb.block(i, i).col(0)<<normal[0], normal[1], normal[2];
        // 	cb.block(i, i).col(1)<<t1[0], t1[1], t1[2];
        // 	cb.block(i, i).col(2)<<t2[0], t2[1], t2[2];

    }
}

argus::DynVec getAdhesions(const vector<arcsim::ArgusImpact> &argusImpacts, double dt) {
    double adhesionValue = 1e-1;
    argus::DynVec adhesions = adhesionValue*argus::DynVec::Ones(argusImpacts.size());
    for (int i = 0; i < argusImpacts.size(); i++) {
        const arcsim::ArgusImpact &impact = argusImpacts[i];
        pair<arcsim::Vec3, arcsim::Vec3> v_and_x1, v_and_x2;
        arcsim::Vec3 xA = impact.nodeA->x;
        arcsim::Vec3 xB;
        arcsim::Vec3 vA = impact.nodeA->v;
        arcsim::Vec3 vB;
        if( impact.nodeB != 0 ){
            xB = impact.nodeB->x;
            vB = impact.nodeB->v;
        } else {
            xB = impact.posB;
            vB = impact.velB;
        }


        arcsim::Vec3 normal = normalize(impact.normal);
        double relativeDistance = dot(xA - xB + vA*dt -vB*dt, normal) - arcsim::magic.projection_thickness;
        // if (relativeDistance > 0) {
        // 	adhesions[i] = 0;
        // } else {
        // double area_scale = impact.nodeB ? (impact.nodeA->a + impact.nodeB->a)*.5 : impact.nodeA->a;
        double area_scale = impact.nodeB ? min(impact.nodeA->a, impact.nodeB->a) : impact.nodeA->a;
        adhesions[i] *= area_scale;
        // }
    }
    return adhesions;
}

void initial_contact_info(const vector<arcsim::ArgusImpact> argusImpacts, arcsim::Mesh &mesh, argus::DynVec &mu,
                          argus::ClothFrictionData::ContactBasisMatrix &cb, argus::DynMat3i &iA, argus::DynMat3 &baryA,
                          argus::DynMat3i &iB, argus::DynMat3 &baryB, argus::DynMat3 &affineVel, const double dt) {
    //vector<Impact> current_impacts = get_current_mesh_impacts(argusImpacts, mesh);
    int contact_number = argusImpacts.size();
    mu.resize(contact_number);
    cb.setRows(contact_number);
    cb.setCols(contact_number);
    iA.resize(3, contact_number);
    baryA.resize(3, contact_number);
    iB.resize(3, contact_number);
    baryB.resize(3, contact_number);
    affineVel.resize(3, contact_number);

    for (int i = 0; i < contact_number; i++) {
        cb.insertBack(i, i);
    }
    cb.finalize();

    for (int i = 0; i < contact_number; i++) {
        const arcsim::ArgusImpact &impact = argusImpacts[i];

        mu[i] = arcsim::magic.friction_coeff;
        pair<arcsim::Vec3, arcsim::Vec3> v_and_x1, v_and_x2;
        arcsim::Vec3 xA;
        arcsim::Vec3 xB;
        arcsim::Vec3 vA;
        arcsim::Vec3 vB;

        iA.col(i) << impact.nodeA -> index , impact.nodeA->index , impact.nodeA->index;  // normally the last two would be like -1
        baryA.col(i) << 1 , 0 , 0;
        xA = impact.nodeA->x;
        vA = impact.nodeA->v;

        // Self-contact case
        if( impact.nodeB != 0 ){
            // std::cout << "self contact case \n";
            iB.col(i) << impact.nodeB -> index , impact.nodeB -> index, impact.nodeB -> index;
            baryB.col(i) << 1 , 0 , 0;
            xB = impact.nodeB->x;
            // vB = impact.nodeB->v;
            vB = arcsim::Vec3(0);
            // Not self-contact (collision is with an external object)
        } else {


            iB.col(i) << -1 , -1 , -1;
            baryB.col(i) << 0 , 0 , 0;
            xB = impact.posB;
            vB = impact.velB;

        }


        arcsim::Vec3 normal = normalize(impact.normal);
        arcsim::Vec3 vCloth = arcsim::Vec3(0);
        arcsim::Vec3 vObs = vB;
        arcsim::Vec3 xCloth = xA;
        arcsim::Vec3 xObs = xB;

        arcsim::Vec3 delX = dot( (xCloth - xObs) , normal )*normal;
        // arcsim::Vec3 delX = xCloth - xObs;

        // arcsim::Vec3 delV = dot(vCloth - vObs, normal)*normal;
        arcsim::Vec3 delV = vCloth - vObs;
        arcsim::Vec3 con;
        // if (dot(delV*dt + delX, normal) < ::magic.projection_thickness) {
        // 	double desired_thickness = min(dot(delV*dt + delX, normal) + 0.1*::magic.projection_thickness, ::magic.projection_thickness);
        // 	con = delV + delX/dt - desired_thickness*normal/dt;  // todo change this
        // } else {
        con = delV + delX/dt - arcsim::magic.projection_thickness*normal/dt;  // todo change this
        // }
        affineVel.col(i)<<con[0], con[1], con[2];

        arcsim::Vec3 t1, t2;

        if ( fabs( dot(normal, arcsim::Vec3(1, 0, 0))) < fabs( dot(normal, arcsim::Vec3(0, 1, 0)))) {
            t1 = cross(normal, arcsim::Vec3(1, 0, 0));
            t2 = cross(normal, t1);
        } else {
            t1 = cross(normal, arcsim::Vec3(0, 1, 0));
            t2 = cross(normal, t1);
        }
        // if( fabs( dot(normal,arcsim::Vec3(1,0,0)) ) < 0.1 || fabs( dot(normal,arcsim::Vec3(1,0,0)) ) > 0.9 ){
        // 	t1 = cross(normal, arcsim::Vec3(0, 1, 0));
        // 	t2 = cross(normal, t1);
        // } else {
        // 	t1 = cross(normal, arcsim::Vec3(1, 0, 0));
        // 	t2 = cross(normal, t1);
        // }
        normal = normalize(normal);

        t1 = normalize(t1);
        t2 = normalize(t2);

        cb.block(i, i).col(0)<<normal[0], normal[1], normal[2];
        cb.block(i, i).col(1)<<t1[0], t1[1], t1[2];
        cb.block(i, i).col(2)<<t2[0], t2[1], t2[2];

    }
}

vector<pair<arcsim::Vec3, arcsim::Vec3> > convert_2_output(const argus::DynVec &v, const argus::DynVec &r) {
    vector<pair<arcsim::Vec3, arcsim::Vec3> > output;
    for (int i = 0; i < v.size()/3; i++) {
        arcsim::Vec3 v_i(v[i*3], v[i*3 + 1], v[i*3 + 2]);
        arcsim::Vec3 r_i(r[i*3], r[i*3 + 1], r[i*3 + 2]);
        output.push_back(make_pair(v_i, r_i));
    }
    return output;
}

vector<arcsim::Vec3> convert_2_output(const argus::DynVec &v) {
    vector<arcsim::Vec3> output;
    for (int i = 0; i < v.size()/3; i++) {
        arcsim::Vec3 v_i(v[i*3], v[i*3 + 1], v[i*3 + 2]);
        output.push_back(v_i);
    }
    return output;
}

vector<pair<arcsim::Vec3, arcsim::Vec3> > BogusIntegrator::solve(const arcsim::SpMat<arcsim::Mat3x3> &A, const vector<arcsim::Vec3> &b, vector<arcsim::Vec3> &linear_v,
                                 const vector<arcsim::ArgusImpact> &argusImpacts,  arcsim::Mesh &mesh, const double dt) {
    argus::ClothFrictionData data;
    argus::ClothFrictionData::StiffnessMatrixType &M = data.M;
    initial_stiff_matrix(A, M);

    argus::DynVec &f = data.f;
    initial_f(b, f);

    argus::DynVec v( data.M.rows() );
    v.setZero();
    if (linear_v.size() > 0) {
        initial_f(linear_v, v);
    }

    argus::DynVec &mu= data.mu;
    argus::ClothFrictionData::ContactBasisMatrix &contactBasis = data.contactBasis;
    argus::DynMat3i &indicesA = data.indicesA;
    argus::DynMat3 &coordsA = data.coordsA;
    argus::DynMat3i &indicesB = data.indicesB;
    argus::DynMat3 &coordsB = data.coordsB;
    argus::DynMat3 &affineVel = data.affineVel;

    initial_contact_info(argusImpacts, mesh, mu, contactBasis, indicesA, coordsA, indicesB, coordsB, affineVel, dt);
    argus::SolverOptions options ;
    options.tolerance = arcsim::magic.argus_tolerance;

    bool scale_by_mass_inv = false;
    argus::DynVec m_inv_vector;
    m_inv_vector.resize(mesh.nodes.size()*3);
    if (scale_by_mass_inv) {
        for (int n = 0; n < mesh.nodes.size(); n++) {
            double m = mesh.nodes[n]->m;
            m_inv_vector.segment(3*n, 3) << 1.0/m, 1.0/m, 1.0/m;
        }
    }

    // static int frame = 0;
    // if (frame++%10 == 0) {
    // data.dump(stringf("%s/frame_%04d", "ignore/benchmarks/short", frame++));
    // }

    // DynVec r( data.M.rows() );
    // r.setZero();
    // DynVec v( data.M.rows() );
    // v.setZero();

    if (!arcsim::magic.facet_solver) {
        data.findAndDuplicate() ;
        v = data.S*v;
    }

    if (arcsim::magic.apply_adhesion) {
        argus::DynVec adhesions = getAdhesions(argusImpacts, dt);
        data.applyAdhesion( adhesions );
    }

    if (scale_by_mass_inv) {
        options.m_inv.clear();
        argus::DynVec extended_m_inv = data.S*m_inv_vector;
        for (int i = 0; i < extended_m_inv.size()/3; i++) {
            options.m_inv.push_back(extended_m_inv.segment(3*i, 3)[0]);
        }
    }

    argus::ClothFrictionSolver solver(data) ;
    argus::SolverStats stats;
    argus::DynVec r( data.M.rows() );
    r.setZero();

    // options.nodalConstraintStepSize = 1e-1;
    if (arcsim::magic.facet_solver) {
        options.algorithm = argus::SolverOptions::Algorithm::ICAGaussSeidel;
        options.faceted = true;
        argus::DynVec last_v(mesh.nodes.size()*3);
        // last_v.setZero();
        for (int n = 0; n < mesh.nodes.size(); n++) {
            arcsim::Node* node = mesh.nodes[n];
            arcsim::Vec3 cur_v = node->v;
            last_v.segment(n*3, 3) << cur_v[0], cur_v[1], cur_v[2];
        }
        data.alignTangentsWithVelocity(last_v);
    } else {
        options.maxOuterIterations = 0;
        options.maxIterations = 2e3;
        options.algorithm = argus::SolverOptions::Algorithm::NodalSelfContact;
    }

    // if (::magic.use_eigen_solver) {
    // 	SpMat<arcsim::Mat3x3> AA;
    // 	vector<arcsim::Vec3> bb;
    // 	reverse_stiff_matrix(data.M, AA);
    // 	reverse_f(data.f, bb);
    // 	vector<arcsim::Vec3> x = eigen_linear_solve(AA, bb);
    // 	initial_f(x, v);
    // 	solver.contactSolve( options, v, r, stats ) ;
    // } else {
    // 	solver.solve( options, v, r, stats ) ;
    // }
    solver.solve(options, v, r, stats);

    // solver.contactSolve( options, v, r, stats);

    arcsim::Log::getInstance()->setError(stats.error);
    arcsim::Log::getInstance()->setContactSolveTime(stats.time);
    arcsim::Log::getInstance()->setIterations(stats.nIterations);

    if (!arcsim::magic.facet_solver) {

        v = data.S.transpose()*v;
        v *= (A_scale/b_scale);
        r = data.S.transpose()*r;
        r *= (1.0/b_scale);
        // Set the contact forces of unconstrained nodes to Zero.
//        bool *in_contact = new bool[r.size()/3];
//        for (int i = 0; i < r.size()/3; i++) {
//            in_contact[i] = false;
//        }
//        for (int i = 0; i < argusImpacts.size(); i++) {
//            const arcsim::ArgusImpact &impact = argusImpacts[i];
//            in_contact[impact.nodeA->index] = true;
//            if (impact.nodeB) {
//                in_contact[impact.nodeB->index] = true;
//            }
//        }
//        for(int i = 0 ; i < r.size()/3 ; ++i) {
//            if (!in_contact[i]) {
//                r.segment(i*3, 3) << 0, 0, 0;
//            }
//        }
//        delete[] in_contact;
    } else {
        argus::DynVec real_r(v.size());
        real_r.setZero();
        for (int i = 0; i < argusImpacts.size(); i++) {
            const arcsim::ArgusImpact &impact = argusImpacts[i];
            int iA = impact.nodeA->index;
            Eigen::Vector3d cur_r = r.segment(i*3, 3);
            real_r.segment(iA*3, 3) += cur_r;
            if (impact.nodeB) {
                int iB = impact.nodeB->index;
                real_r.segment(iB*3, 3) -= cur_r;
            }
        }
        r = real_r;
    }


    return convert_2_output(v, r);
}

vector<pair<arcsim::Vec3, arcsim::Vec3> > BogusIntegrator::solve(const arcsim::SpMat<arcsim::Mat3x3> &A, const vector<arcsim::Vec3> &b, vector<arcsim::Vec3> &linear_v,
                                const vector<arcsim::Impact> &impacts,  arcsim::Mesh &mesh, const double dt) {
    argus::ClothFrictionData data;
    argus::ClothFrictionData::StiffnessMatrixType &M = data.M;
    initial_stiff_matrix(A, M);

    argus::DynVec &f = data.f;
    initial_f(b, f);

    argus::DynVec &mu= data.mu;
    argus::ClothFrictionData::ContactBasisMatrix &contactBasis = data.contactBasis;
    argus::DynMat3i &indicesA = data.indicesA;
    argus::DynMat3 &coordsA = data.coordsA;
    argus::DynMat3i &indicesB = data.indicesB;
    argus::DynMat3 &coordsB = data.coordsB;
    argus::DynMat3 &affineVel = data.affineVel;

    initial_contact_info(impacts, mesh, mu, contactBasis, indicesA, coordsA, indicesB, coordsB, affineVel, dt);
    argus::SolverOptions options ;
    // options.tolerance = 1.e-12;
    // options.maxOuterIterations = 0;

    options.algorithm = (argus::SolverOptions::Algorithm)5;

    // DynVec r( data.M.rows() );
    // r.setZero();
    // DynVec v( data.M.rows() );
    // v.setZero();
    data.findAndDuplicate() ;
    argus::ClothFrictionSolver solver(data) ;
    argus::SolverStats stats;
    argus::DynVec extended_v( data.M.rows() );
    extended_v.setZero();
    argus::DynVec extended_r( data.M.rows() );
    extended_r.setZero();

    static bool first = true;


    // static int frame = 0;

    // data.dump(stringf("%s_%02d.obj", "Problems/simulate", frame++));
    solver.solve( options, extended_v, extended_r, stats ) ;

    arcsim::Log::getInstance()->setError(stats.error);
    arcsim::Log::getInstance()->setContactSolveTime(stats.time);
    arcsim::Log::getInstance()->setIterations(stats.nIterations);

    argus::DynVec v = data.pinvS*extended_v;
    argus::DynVec r = data.pinvS*extended_r;

    // Set the contact forces of unconstrained nodes to Zero.
    // bool *in_contact = new bool[r.size()/3];
    // for (int i = 0; i < r.size()/3; i++) {
    // 	in_contact[i] = false;
    // }
    // for (int i = 0; i < argusImpacts.size(); i++) {
    // 	const ArgusImpact &impact = argusImpacts[i];
    // 	in_contact[impact.nodeA->index] = true;
    // 	if (impact.nodeB) {
    // 		in_contact[impact.nodeB->index] = true;
    // 	}
    // }
    // for(int i = 0 ; i < r.size()/3 ; ++i) {
    // 	if (!in_contact[i]) {
    // 		r.segment(i*3, 3) << 0, 0, 0;
    // 	}
    // }
    return convert_2_output(v, r);
}

vector<arcsim::Vec3> BogusIntegrator::linear_solve(const arcsim::SpMat<arcsim::Mat3x3> &A, const vector<arcsim::Vec3> &b) {
    argus::ClothFrictionData data;
    argus::ClothFrictionData::StiffnessMatrixType &M = data.M;
    initial_stiff_matrix(A, M);

    argus::DynVec &f = data.f;
    initial_f(b, f);

    argus::ClothFrictionSolver solver(data) ;
    argus::DynVec v( data.M.rows() );
    solver.linearSolve(v);

    return convert_2_output(v);
}