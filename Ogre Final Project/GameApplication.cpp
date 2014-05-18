/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *        _______              __     __                     *
 *       / _____/             / /    / /                     *
 *      / /__ _____   ______ / /___ / /__  ______ __  __     *
 *     / ___// __  | / ____// __  // //_/ / __  // /_/ /     *
 *    / /   / /_/ /_|__  | / / / // / \  / ____//___  /      *
 *   /_/   |_____//_____/ /_/ /_//_/ \_\/_____//_____/       *
 *                                                           *
 * This source file is developed by Joshua J. Nelson         *
 * Copyright ©2013 Foshkey Productions, foshkey@gmail.com    *
 *                                                           *
 * Permission is hereby granted, free of charge, to any      *
 * person obtaining a copy of this software and associated   *
 * documentation files (the "Software"), to deal in the      *
 * Software without restriction, including without           *
 * limitation the rights to use, copy, modify, merge,        *
 * publish, distribute, sublicense, and/or sell copies of    *
 * the Software, and to permit persons to whom the Software  *
 * is furnished to do so, subject to the following           *
 * conditions:                                               *
 *                                                           *
 * The above copyright notice and this permission notice     *
 * shall be included in all copies or substantial portions   *
 * of the Software.                                          *
 *                                                           *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY *
 * KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO    *
 * THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A          *
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL *
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, *
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF       *
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN   *
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS *
 * IN THE SOFTWARE.                                          *
 *                                                           *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//=============================================================
// SOURCES
// Ogre Tutorials:      http://www.ogre3d.org/tikiwiki/tiki-index.php?page=Tutorials
// Bullet Tutorial:     http://www.ogre3d.org/tikiwiki/OgreBullet
// Audiere:             http://www.ogre3d.org/forums/viewtopic.php?t=18204&highlight=&sid=ce193664e1d3d7c4af509e6f4e2718c6

//=============================================================
// Included Libraries
#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>
#include <CEGUISystem.h>
#include <CEGUISchemeManager.h>
#include <RendererModules/Ogre/CEGUIOgreRenderer.h>
#include <Shapes/OgreBulletCollisionsTerrainShape.h>
#include <OgreBulletDynamicsRigidBody.h>
#include <Shapes/OgreBulletCollisionsStaticPlaneShape.h>
#include <Shapes/OgreBulletCollisionsBoxShape.h>
#include <audiere.h>

//=============================================================
// Included Files
#include "GameApplication.h"

//=============================================================
// Namespaces
using namespace Ogre;

//=============================================================
// Constructor / Destructor
//=============================================================
GameApplication::GameApplication(void)
    :   mTerrainGlobals(0),
        mTerrainGroup(0),
        mTerrainsImported(false),
        mInfoLabel(0),
        mCamHeight(6),
        mCamHeightVelocity(0),
        bLMouseDown(false),
        bRMouseDown(false)
{

}
//=============================================================
GameApplication::~GameApplication(void)
{
    // OgreBullet physic delete - RigidBodies
 	std::deque<OgreBulletDynamics::RigidBody *>::iterator itBody = mBodies.begin();
 	while (mBodies.end() != itBody)
 	{   
 		delete *itBody; 
 		++itBody;
 	}	
 	// OgreBullet physic delete - Shapes
 	std::deque<OgreBulletCollisions::CollisionShape*>::iterator itShape = mShapes.begin();
 	while (mShapes.end() != itShape)
 	{   
 		delete *itShape; 
 		++itShape;
 	}
 	mBodies.clear();
 	mShapes.clear();
 	delete mWorld->getDebugDrawer();
 	mWorld->setDebugDrawer(0);
 	delete mWorld;
}
//=============================================================
void GameApplication::destroyScene(void)
{
    OGRE_DELETE mTerrainGroup;
    OGRE_DELETE mTerrainGlobals;
}

//=============================================================
// Terrain methods
//=============================================================
void getTerrainImage( bool flipX, bool flipY, Image& img )
{
    img.load( "terrain.png", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    if ( flipX )
        img.flipAroundY();
    if ( flipY )
        img.flipAroundX();
}
//=============================================================
void GameApplication::defineTerrain( long x, long y )
{
    String filename = mTerrainGroup->generateFilename( x, y );
    
    if ( ResourceGroupManager::getSingleton().resourceExists( mTerrainGroup->getResourceGroup(), filename ) )
    {
        mTerrainGroup->defineTerrain( x, y );
    }
    else
    {
        Image img;
        getTerrainImage( x % 2 != 0, y % 2 != 0, img );
        mTerrainGroup->defineTerrain( x, y, &img );
        mTerrainsImported = true;
    }

}
//=============================================================
void GameApplication::initBlendMaps( Terrain* terrain )
{
    TerrainLayerBlendMap* blendMap0 = terrain->getLayerBlendMap(1);
    TerrainLayerBlendMap* blendMap1 = terrain->getLayerBlendMap(2);

    Real minHeight0 = 70;
    Real fadeDist0  = 40;
    Real minHeight1 = 70;
    Real fadeDist1  = 15;

    float* pBlend0 = blendMap0->getBlendPointer();
    float* pBlend1 = blendMap1->getBlendPointer();

    for ( uint16 y = 0; y < terrain->getLayerBlendMapSize(); y++ )
    {
        for ( uint16 x = 0; x < terrain->getLayerBlendMapSize(); x++ )
        {
            Real tx, ty;

            blendMap0->convertImageToTerrainSpace( x, y, &tx, &ty );
            Real height = terrain->getHeightAtTerrainPosition( tx, ty );
            Real val = ( height - minHeight0 ) / fadeDist0;
            val = Math::Clamp( val, (Real)0, (Real)1 );
            *pBlend0++ = val;

            val = ( height - minHeight1 ) / fadeDist1;
            val = Math::Clamp( val, (Real)0, (Real)1 );
            *pBlend1++ = val;
        }
    }

    blendMap0->dirty();
    blendMap1->dirty();
    blendMap0->update();
    blendMap1->update();
}
//=============================================================
// NOTES ON THIS METHOD: initTerrainCollision
// OgreBullet does not properly implement terrain collision,
// So, I cannot claim this code is my own creation. The Author,
// captaincrunch80, uses native bullet to initialize terrain
// collision in the physics world.
// source: http://www.ogre3d.org/addonforums/viewtopic.php?f=12&t=14583
//=============================================================
void GameApplication::initTerrainCollision( Terrain* terrain )
{
    int terrainPageSize = terrain->getSize();

    // Convert to Bullet Height Data
    float *pTerrainHeightData = terrain->getHeightData();
    float *pTerrainHeightDataConvert = (float*)calloc(terrainPageSize * terrainPageSize, sizeof(float));
    for(int i = 0; i < terrainPageSize; ++i)
    {
        memcpy(pTerrainHeightDataConvert + terrainPageSize * i,
                pTerrainHeightData + terrainPageSize * (terrainPageSize - i - 1),
                sizeof(float)*(terrainPageSize));
    }

    // Create Shape
    btHeightfieldTerrainShape* pHeightShape
        = new btHeightfieldTerrainShape( terrainPageSize,
                                         terrainPageSize,
                                         pTerrainHeightDataConvert,
                                         1,
                                         terrain->getMinHeight(),
                                         terrain->getMaxHeight(),
                                         1,
                                         PHY_FLOAT,
                                         true );

    // Scale the mesh along x/z
    float unitsBetweenVertices = terrain->getWorldSize() / ( terrainPageSize - 1 );
    btVector3 scaling( unitsBetweenVertices, 1, unitsBetweenVertices );
    pHeightShape->setLocalScaling(scaling);

    // Ogre uses DiamonSubdivision for Terrain-mesh, so bullet should use it too
    pHeightShape->setUseDiamondSubdivision(true);

    // Create a btRigidBody
    btRigidBody *pBody = new btRigidBody(0.0, new btDefaultMotionState(), pHeightShape);

    Vector3 terrainPosition = terrain->getPosition();
    pBody->getWorldTransform().setOrigin( btVector3( terrainPosition.x,
                                                     terrainPosition.y 
                                                     + ( terrain->getMaxHeight() - terrain->getMinHeight() ) / 2,
                                                     terrainPosition.z ) );

    pBody->getWorldTransform().setRotation( btQuaternion( Quaternion::IDENTITY.x,
                                                          Quaternion::IDENTITY.y,
                                                          Quaternion::IDENTITY.z,
                                                          Quaternion::IDENTITY.w ) );

    // Natively add the terrain rigid body
    mWorld->getBulletDynamicsWorld()->addRigidBody( pBody );
}
//=============================================================
void GameApplication::configureTerrainDefaults( Light* light )
{
    // Configure global
    mTerrainGlobals->setMaxPixelError(8);

    // Testing composite map
    mTerrainGlobals->setCompositeMapDistance(3000);

    // Important to set these so that the terrain knows what to use for derived (non-realtime) data
    mTerrainGlobals->setLightMapDirection( light->getDerivedDirection() );
    mTerrainGlobals->setCompositeMapAmbient( mSceneMgr->getAmbientLight() );
    mTerrainGlobals->setCompositeMapDiffuse( light->getDiffuseColour() );

    // Configure default import settings for if we use imported image
    Terrain::ImportData& defaultimp = mTerrainGroup->getDefaultImportSettings();
    defaultimp.terrainSize  = 513;
    defaultimp.worldSize    = 12000.0f;
    defaultimp.inputScale   = 600;
    defaultimp.minBatchSize = 33;
    defaultimp.maxBatchSize = 65;

    // Textures
    defaultimp.layerList.resize(3);
    defaultimp.layerList[0].worldSize = 100;
    defaultimp.layerList[0].textureNames.push_back( "dirt_grayrocky_diffusespecular.dds" );
    defaultimp.layerList[0].textureNames.push_back( "dirt_grayrocky_normalheight.dds" );
    defaultimp.layerList[1].worldSize = 30;
    defaultimp.layerList[1].textureNames.push_back( "grass_green-01_diffusespecular.dds" );
    defaultimp.layerList[1].textureNames.push_back( "grass_green-01_normalheight.dds" );
    defaultimp.layerList[2].worldSize = 200;
    defaultimp.layerList[2].textureNames.push_back( "growth_weirdfungus-03_diffusespecular.dds" );
    defaultimp.layerList[2].textureNames.push_back( "growth_weirdfungus-03_normalheight.dds" );
}
//=============================================================
// Bullet methods
//=============================================================
void GameApplication::createBox( Vector3 position, Vector3 velocity )
{
    Vector3 size = Vector3::ZERO;	// size of the box
 
 	// create an ordinary, Ogre mesh with texture
 	Entity *entity = mSceneMgr->createEntity(
 			"Box" + StringConverter::toString(mNumEntitiesInstanced),
 			"cube.mesh");
 	entity->setCastShadows(true);
 	// we need the bounding box of the box to be able to set the size of the Bullet-box
 	AxisAlignedBox boundingB = entity->getBoundingBox();
 	size = boundingB.getSize(); size /= 2.0f; // only the half needed
 	size *= 0.95f;	// Bullet margin is a bit bigger so we need a smaller size
 							// (Bullet 2.76 Physics SDK Manual page 18)
 	entity->setMaterialName("Examples/BumpyMetal");
 	SceneNode *node = mSceneMgr->getRootSceneNode()->createChildSceneNode();
 	node->attachObject(entity);
 	node->scale(0.05f, 0.05f, 0.05f);	// the cube is too big for us
 	size *= 0.05f;						// don't forget to scale down the Bullet-box too
 
 	// after that create the Bullet shape with the calculated size
 	OgreBulletCollisions::BoxCollisionShape *sceneBoxShape = new OgreBulletCollisions::BoxCollisionShape(size);
 	// and the Bullet rigid body
 	OgreBulletDynamics::RigidBody *defaultBody = new OgreBulletDynamics::RigidBody(
 			"defaultBoxRigid" + StringConverter::toString(mNumEntitiesInstanced), 
 			mWorld);
 	defaultBody->setShape(	node,
 				sceneBoxShape,
 				0.6f,			// dynamic body restitution
 				0.6f,			// dynamic body friction
 				1.0f, 			// dynamic bodymass
 				position,		// starting position of the box
 				Quaternion(0,0,0,1));// orientation of the box
 	mNumEntitiesInstanced++;				
 
 	defaultBody->setLinearVelocity( velocity ); // shooting speed
 	// push the created objects to the dequese
 	mShapes.push_back(sceneBoxShape);
 	mBodies.push_back(defaultBody);
}
//=============================================================
// Audiere methods
//=============================================================
float GameApplication::get3DFactor( Vector3 listener, Vector3 source )
{
    float ret = 0.0f;
    float dist = listener.distance( source );

    ret = ( -1.0f / 490.0f ) * dist + (50.0f / 49.0f);

    if ( ret > 1 ) ret = 1;
    if ( ret < 0 ) ret = 0;

    return ret;
}
//=============================================================
// Camera methods
//=============================================================
void GameApplication::setCameraHeight( const FrameEvent& evt )
{
    Vector3 camPos = mCamera->getPosition();
    float terrainHeight = mTerrainGroup->getHeightAtWorldPosition( camPos );
    float newHeight = camPos.y;

    mCamHeightVelocity += evt.timeSinceLastFrame * -2; // replace with gravity variable
    newHeight += mCamHeightVelocity;

    if ( newHeight <= terrainHeight + 6.0f )
    {
        mCamHeightVelocity = 0;
        newHeight = terrainHeight + 6.0f;
    }

    mCamera->setPosition(camPos.x, newHeight, camPos.z);
}

//=============================================================
// Init methods
//=============================================================
void GameApplication::initCamera(void)
{
    mCamera->setPosition( Vector3( 1683, 50, 2116 ) );
    mCamera->lookAt( Vector3( 1963, 50, 1660 ) );
    mCamera->setNearClipDistance(0.1);
    mCamera->setFarClipDistance(50000);
    if ( mRoot->getRenderSystem()->getCapabilities()->hasCapability( RSC_INFINITE_FAR_PLANE ) )
    {
        mCamera->setFarClipDistance(0);
    }
}
//=============================================================
void GameApplication::initLight(void)
{
    Vector3 lightdir( 0.55, -0.3, 0.75 );
    lightdir.normalise();

    Light* light = mSceneMgr->createLight( "tstLight" );
    light->setType( Light::LT_DIRECTIONAL );
    light->setDirection( lightdir );
    light->setDiffuseColour( ColourValue::White );
    light->setSpecularColour( ColourValue( 0.4, 0.4, 0.4 ) );

    mSceneMgr->setAmbientLight(ColourValue(0.2, 0.2, 0.2));
}
//=============================================================
void GameApplication::initTerrain(void)
{
    mTerrainGlobals = OGRE_NEW TerrainGlobalOptions();
    mTerrainGroup = OGRE_NEW TerrainGroup( mSceneMgr, Terrain::ALIGN_X_Z, 513, 12000.0f );
    mTerrainGroup->setFilenameConvention( String( "GameApplicationTerrain" ), String( "dat" ) );
    mTerrainGroup->setOrigin( Vector3::ZERO );

    Light* light = mSceneMgr->getLight( "tstLight" );
    configureTerrainDefaults( light );
 
    for ( long x = 0; x <= 0; x++ )
        for ( long y = 0; y <= 0; y++ )
            defineTerrain( x, y );
 
    mTerrainGroup->loadAllTerrains(true);
 
    if ( mTerrainsImported )
    {
        TerrainGroup::TerrainIterator ti = mTerrainGroup->getTerrainIterator();
        while( ti.hasMoreElements() )
        {
            Terrain* t = ti.getNext()->instance;
            initBlendMaps(t);
        }
    }
 
    mTerrainGroup->freeTemporaryResources();
}
//=============================================================
void GameApplication::initSky(void)
{
    mSceneMgr->setSkyDome(true, "Examples/CloudySky", 5, 8);
}
//=============================================================
void GameApplication::initFog(void)
{
    ColourValue fadeColour( 0.9, 0.9, 0.9 );
    mSceneMgr->setFog( FOG_LINEAR, fadeColour, 0.0, 1000, 2000 );
    mWindow->getViewport(0)->setBackgroundColour( fadeColour );
}
//=============================================================
void GameApplication::initCursor(void)
{
    //CEGUI setup
	mGUIRenderer = &CEGUI::OgreRenderer::bootstrapSystem();
 
	//show the CEGUI cursor
	CEGUI::SchemeManager::getSingleton().create((CEGUI::utf8*)"TaharezLook.scheme");
	CEGUI::MouseCursor::getSingleton().setImage("TaharezLook", "MouseArrow");
}
//=============================================================
void GameApplication::initBullet(void)
{
    // Initialize Bullet world
    mNumEntitiesInstanced = 0;
    gravityVector = Vector3( 0, -9.81, 0 );
    bounds = AxisAlignedBox( Vector3 (-10000, -10000, -10000), Vector3 (10000,  10000,  10000) );
    mWorld = new OgreBulletDynamics::DynamicsWorld( mSceneMgr, bounds, gravityVector );
    debugDrawer = new OgreBulletCollisions::DebugDrawer();
    debugDrawer->setDrawWireframe(true);
    mWorld->setDebugContactPoints(debugDrawer);
    mWorld->setShowDebugShapes(true);

    // Create node
    SceneNode *node = mSceneMgr->getRootSceneNode()->createChildSceneNode("debugDrawer", Vector3::ZERO);
    node->attachObject( static_cast<SimpleRenderable *> (debugDrawer) );

    MaterialManager::getSingleton().setDefaultTextureFiltering(TFO_ANISOTROPIC);
    MaterialManager::getSingleton().setDefaultAnisotropy(7);

    // Set Terrain Collision
    TerrainGroup::TerrainIterator ti = mTerrainGroup->getTerrainIterator();
    while( ti.hasMoreElements() )
    {
        Terrain* t = ti.getNext()->instance;
        initTerrainCollision(t);
    }
}
//=============================================================
void GameApplication::initAudiere(void)
{
    device = audiere::AudioDevicePtr( audiere::OpenDevice() );

    boxSound = audiere::SoundEffectPtr( audiere::OpenSoundEffect( device, "EpicMetal_Hard.wav", audiere::SoundEffectType::MULTIPLE ) );

    song = audiere::OutputStreamPtr( audiere::OpenSound( device, "01 - The Cell.mp3", false ) );
    song->setVolume( 0.5f );
    song->setRepeat( true );
    song->play();
}
//=============================================================
// Protected methods
//=============================================================
void GameApplication::createScene(void)
{
    initCamera();
    initLight();
    initTerrain();
    //initFog();
    initSky();
    initCursor();
    initBullet();
    initAudiere();
}
//=============================================================
void GameApplication::createFrameListener(void)
{
    BaseApplication::createFrameListener();

    mInfoLabel = mTrayMgr->createLabel( OgreBites::TL_TOP, "TInfo", "", 350 );
}
//=============================================================
bool GameApplication::frameStarted(const FrameEvent& evt)
{
    mWorld->stepSimulation(evt.timeSinceLastFrame);
    return true;
}
//=============================================================
bool GameApplication::frameEnded(const FrameEvent& evt)
{
    mWorld->stepSimulation(evt.timeSinceLastFrame);
    return true;
}
//=============================================================
bool GameApplication::frameRenderingQueued( const FrameEvent& evt )
{
    bool ret = BaseApplication::frameRenderingQueued( evt );

    if ( mTerrainGroup->isDerivedDataUpdateInProgress() )
    {
        mTrayMgr->moveWidgetToTray( mInfoLabel, OgreBites::TL_TOP, 0 );
        mInfoLabel->show();
        if ( mTerrainsImported )
        {
            mInfoLabel->setCaption( "Building terrain, please wait..." );
        }
        else
        {
            mInfoLabel->setCaption( "Updating textures, patience..." );
        }
    }
    else
    {
        mTrayMgr->removeWidgetFromTray( mInfoLabel );
        mInfoLabel->hide();
        if ( mTerrainsImported )
        {
            mTerrainGroup->saveAllTerrains(true);
            mTerrainsImported = false;
        }
    }

    // Camera Height
    setCameraHeight(evt);

    return ret;
}
//=============================================================
bool GameApplication::keyPressed( const OIS::KeyEvent& arg )
{
    bool ret = BaseApplication::keyPressed( arg );

    if ( arg.key == OIS::KC_SPACE )
    {
        mCamHeightVelocity = 0.5; // replace with jumping variable
    }
    if ( arg.key == OIS::KC_B )
 	{
        createBox( mCamera->getDerivedPosition() + mCamera->getDerivedDirection().normalisedCopy() * 10,
                   mCamera->getDerivedDirection().normalisedCopy() * 100.0f );
 	}
    
    return ret;
}

//=============================================================
bool GameApplication::mousePressed( const OIS::MouseEvent& arg, OIS::MouseButtonID id)
{
    if ( id == OIS::MB_Left )
    {
        bLMouseDown = true;

        // Mouse Ray
        CEGUI::Point mousePos = CEGUI::MouseCursor::getSingleton().getPosition();
        Ray mouseRay = mCamera->getCameraToViewportRay( mousePos.d_x / (float)arg.state.width, mousePos.d_y / (float)arg.state.height );
        TerrainGroup::RayResult rayResult = mTerrainGroup->rayIntersects( mouseRay );

        // Spawn Box if hit terrain
        if ( rayResult.hit )
        {
            Vector3 boxPos = rayResult.position;
            boxPos.y += 5;
            createBox( boxPos, Vector3::ZERO );

            boxSound->setVolume( 0.4f * get3DFactor( mCamera->getDerivedPosition(), boxPos ) );
            boxSound->play();
        }
    }
    else if ( id == OIS::MB_Right )
	{
		CEGUI::MouseCursor::getSingleton().hide();
		bRMouseDown = true;
	}
    return true;
}

//=============================================================
bool GameApplication::mouseReleased( const OIS::MouseEvent& arg, OIS::MouseButtonID id)
{
    if ( id == OIS::MB_Left )
    {
        bLMouseDown = false;
    }
    else if ( id == OIS::MB_Right )
	{
		CEGUI::MouseCursor::getSingleton().show();
		bRMouseDown = false;
	}
    return true;
}

//=============================================================
bool GameApplication::mouseMoved( const OIS::MouseEvent& arg )
{
    CEGUI::System::getSingleton().injectMouseMove(arg.state.X.rel, arg.state.Y.rel);

    if(bRMouseDown)	//if the right mouse button is held down, be rotate the camera with the mouse
	{
		mCamera->yaw( Degree(-arg.state.X.rel * 0.5f) );
		mCamera->pitch( Degree(-arg.state.Y.rel * 0.5f) );
	}
 
	return true;
}

//=============================================================
// Main Execution
//=============================================================
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
    int main(int argc, char *argv[])
#endif
    {
        // Create application object
        GameApplication app;

        try {
            app.go();
        } catch( Ogre::Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
            MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
            std::cerr << "An exception has occured: " <<
                e.getFullDescription().c_str() << std::endl;
#endif
        }

        return 0;
    }

#ifdef __cplusplus
}
#endif
