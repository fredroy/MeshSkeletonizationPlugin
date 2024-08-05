/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
/*
 * CGALSkeltonization.cpp
 *
 * Author: Nazim Haouchine & Sidaty El Hadramy
 */

#pragma once

#include <MeshSkeletonizationPlugin/MeshSkeletonization.h>

using namespace sofa::core::objectmodel;

namespace meshskeletonizationplugin
{

template <class DataTypes>
MeshSkeletonization<DataTypes>::MeshSkeletonization()
    : d_inVertices(initData (&d_inVertices, "inputVertices", "List of input mesh vertices"))
    , d_inTriangles(initData(&d_inTriangles, "inputTriangles", "List of input mesh triangles"))
    , d_outSkeletonFilename(initData(&d_outSkeletonFilename, "outputSkeleton", "File path to output skeleton file"))
{
    addInput(&d_inVertices);
    addInput(&d_inTriangles);

    addOutput(&d_outSkeletonFilename);
}


template <class DataTypes>
void MeshSkeletonization<DataTypes>::init() 
{
    //Input
    if(d_outSkeletonFilename.getValue().empty())
    {
        msg_error() << "No input File to store the skeleton data, please set a inputFile path.";
        return;
    }
}


template <class DataTypes>
void MeshSkeletonization<DataTypes>::geometryToPolyhedron(Polyhedron &s)
{
    VecCoord inVertices = d_inVertices.getValue();
    SeqTriangles inTriangles = d_inTriangles.getValue();

    geometryToPolyhedronOp<HalfedgeDS> gen(inVertices, inTriangles);
    s.delegate(gen);
}


template <class DataTypes>
void MeshSkeletonization<DataTypes>::doUpdate() 
{
    Polyhedron tmesh;

    geometryToPolyhedron(tmesh);
    msg_info() << "Number of vertices of the input mesh: " << boost::num_vertices(tmesh);

    if (!CGAL::is_triangle_mesh(tmesh))
    {
        msg_error() << "Input geometry is not triangulated.";
        return;	
    }
        
    CGAL::extract_mean_curvature_flow_skeleton(tmesh, m_skeleton);
    msg_info() << "Number of vertices of the output skeleton: " << boost::num_vertices(m_skeleton);
    msg_info() << "Number of edges of the output skeleton: " << boost::num_edges(m_skeleton);

    if (d_outSkeletonFilename.isSet())
    {
        std::ofstream OutputP(d_outSkeletonFilename.getFullPath(), std::ofstream::out | std::ofstream::trunc);
        Export_polylines extractor(m_skeleton, OutputP);
        CGAL::split_graph_into_polylines(m_skeleton, extractor);
        OutputP.close();
    }
}


template <class DataTypes>
void MeshSkeletonization<DataTypes>::draw(const sofa::core::visual::VisualParams* vparams) 
{
    using Color = sofa::type::RGBAColor;
    std::vector< type::Vec3 > dvec;

    for(const Skeleton_edge& e : CGAL::make_range(edges(m_skeleton))) 
    {
        const Point& s = m_skeleton[source(e, m_skeleton)].point;
        const Point& t = m_skeleton[target(e, m_skeleton)].point;

                
        dvec.emplace_back(Coord(s[0], s[1], s[2]));
        dvec.emplace_back(Coord(t[0], t[1], t[2]));

        vparams->drawTool()->drawLines( dvec, 40, Color::red());
        dvec.clear();
    } 
}

} // namespace meshskeletonizationplugin
