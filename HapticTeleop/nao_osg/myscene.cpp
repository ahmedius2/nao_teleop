#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/Geode>
#include <osg/MatrixTransform> // to scale sphere into an ellipsoid
#include <osgDB/WriteFile>
#include <iostream>

// The osg::Geode class corresponds to the leaf node of a scene graph.
// It stores Drawable object pointers

// OSG provides an osg::ShapeDrawable class, which inherits from the osg::Drawable
// base class, to render basic geometry shapes quickly with plain parameters.

// I should use osg::Geometry insead of osg::ShapeDrawable !


/*
int main( int argc, char** argv )
{
	// ref_ptr is smart ptr
	osg::ref_ptr<osg::Node> root = osgDB::readNodeFile("bounceball.osg");
	osgViewer::Viewer viewer;
	viewer.setSceneData( root.get() );
	return viewer.run();
}

int main( int argc, char** argv )
{

	// inefficient but more straightforward way
	osg::ref_ptr<osg::ShapeDrawable> shape2 = new osg::ShapeDrawable;
	shape2->setShape( new osg::Sphere(osg::Vec3(3.0f, 0.0f, 0.0f),
		1.0f) );
	shape2->setColor( osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f) );

	// faster way, using osg::Geometry and defining how to draw it using primitives
	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	vertices->push_back( osg::Vec3(0.0f, 0.0f, 0.0f) );
	vertices->push_back( osg::Vec3(1.0f, 0.0f, 0.0f) );
	vertices->push_back( osg::Vec3(1.0f, 0.0f, 1.0f) );
	vertices->push_back( osg::Vec3(0.0f, 0.0f, 1.0f) );

	// I think the normal defines the direction that vertex looks
	// If it looks to the source of the light it will be seen perfectly
	// if not, it will be dark
	osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
	normals->push_back( osg::Vec3(0.0f,-1.0f, 0.0f) );

	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back( osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f) );
	colors->push_back( osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) );
	colors->push_back( osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f) );
	colors->push_back( osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f) );

	osg::ref_ptr<osg::Geometry> quad = new osg::Geometry;
	quad->setVertexArray( vertices.get() );
	quad->setNormalArray( normals.get() );
	quad->setNormalBinding( osg::Geometry::BIND_OVERALL );
	quad->setColorArray( colors.get() );
	quad->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
	quad->addPrimitiveSet( new osg::DrawArrays(GL_QUADS, 0, 4) );


	osg::ref_ptr<osg::Geode> root = new osg::Geode;
	root->addDrawable( shape2.get() ); 
	root->addDrawable( quad.get() );

	osgViewer::Viewer viewer;
	viewer.setSceneData( root.get() );
	return viewer.run();
}

*/

int main( int argc, char** argv )
{
	osg::ref_ptr<osg::ShapeDrawable> shape2 = new osg::ShapeDrawable;
	shape2->setShape( new osg::Sphere(osg::Vec3(3.0f, 0.0f, 0.0f),
		1.0f) );
	shape2->setColor( osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f) );

	osg::ref_ptr<osg::Geode> ellipseGeode = new osg::Geode;
	ellipseGeode->addDrawable( shape2.get() ); 

	osg::ref_ptr<osg::MatrixTransform> root = new osg::MatrixTransform;
	root->setMatrix(osg::Matrix::scale(2,1,4));
	root->addChild(ellipseGeode.get());

	if(!osgDB::writeNodeFile(*(root.get()), "manip_ellipsoid.osg")){
		std::cout << "osg couldn't write node graph to file!" << std::endl;
		return -1;
	}

	osgViewer::Viewer viewer;
	viewer.setSceneData( root.get() );
	return viewer.run();
}
