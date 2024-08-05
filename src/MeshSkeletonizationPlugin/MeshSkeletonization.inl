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
    , d_outVertices(initData(&d_outVertices, "outputVertices", "List of output skeleton vertices"))
    , d_outEdges(initData(&d_outEdges, "outputEdges", "List of output skeleton edges"))
    , d_outSkeletonFilename(initData(&d_outSkeletonFilename, "outputSkeleton", "File path to output skeleton file"))
{
    addInput(&d_inVertices);
    addInput(&d_inTriangles);

    addOutput(&d_outVertices);
    addOutput(&d_outEdges);
    addOutput(&d_outSkeletonFilename);
}


template <class DataTypes>
void MeshSkeletonization<DataTypes>::init() 
{
    // trigger update
    this->doUpdate();
}


template <class DataTypes>
void MeshSkeletonization<DataTypes>::geometryToPolyhedron(Polyhedron &s)
{
    const VecCoord& inVertices = d_inVertices.getValue();
    const SeqTriangles& inTriangles = d_inTriangles.getValue();

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
    const auto skeletonNbVertices = boost::num_vertices(m_skeleton);
    const auto skeletonNbEdges = boost::num_edges(m_skeleton);
    msg_info() << "Number of vertices of the output skeleton: " << skeletonNbVertices;
    msg_info() << "Number of edges of the output skeleton: " << skeletonNbEdges;

    // Convert CGAL data to SOFA data (and store them)
    {
        auto sofaOutVertices = sofa::helper::getWriteOnlyAccessor(d_outVertices);
        auto sofaOutEdges = sofa::helper::getWriteOnlyAccessor(d_outEdges);
        sofaOutVertices.clear();
        sofaOutEdges.clear();
        sofaOutVertices.resize(skeletonNbVertices);
        sofaOutEdges.reserve(skeletonNbEdges);

        for (const Skeleton_edge& e : CGAL::make_range(edges(m_skeleton)))
        {
            const auto e0 = source(e, m_skeleton);
            const auto e1 = target(e, m_skeleton);

            const Point& s = m_skeleton[e0].point;
            const Point& t = m_skeleton[e1].point;

            sofaOutVertices[e0] = { s[0], s[1], s[2] };
            sofaOutVertices[e1] = { t[0], t[1], t[2] };

            sofaOutEdges.emplace_back(e0, e1);
        }
    }

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
    std::vector< type::Vec3 > vertices;

    auto sofaOutVertices = sofa::helper::getReadAccessor(d_outVertices);
    auto sofaOutEdges = sofa::helper::getReadAccessor(d_outEdges);
    vertices.reserve(sofaOutEdges.size() * 2);

    [[maybe_unused]] auto state = vparams->drawTool()->makeStateLifeCycle();
    vparams->drawTool()->disableLighting();

    for(const auto& e : sofaOutEdges)
    {
        vertices.emplace_back(DataTypes::getCPos(sofaOutVertices[e[0]]));
        vertices.emplace_back(DataTypes::getCPos(sofaOutVertices[e[1]]));
    }
    vparams->drawTool()->drawLines(vertices, 1, Color::red());

}

} // namespace meshskeletonizationplugin
