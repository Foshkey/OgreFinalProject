#ifndef __GameApplication_h_
#define __GameApplication_h_

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
// Included Libraries
#include <Terrain/OgreTerrain.h>
#include <Terrain/OgreTerrainGroup.h>
#include <CEGUISystem.h>
#include <CEGUISchemeManager.h>
#include <RendererModules/Ogre/CEGUIOgreRenderer.h>
#include <OgreBulletDynamicsRigidBody.h>
#include <audiere.h>

//=============================================================
// Included Files
#include "BaseApplication.h"

//=============================================================
// Namespaces
using namespace Ogre;

//=============================================================
//=============================================================
class GameApplication : public BaseApplication
{
private:
    // Terrain
    TerrainGlobalOptions* mTerrainGlobals;
    TerrainGroup* mTerrainGroup;
    bool mTerrainsImported;
    OgreBites::Label* mInfoLabel;
    void defineTerrain( long x, long y );
    void initBlendMaps( Terrain* terrain );
    void initTerrainCollision( Terrain* terrain );
    void configureTerrainDefaults( Light* light );

    // CEGUI
    CEGUI::Renderer* mGUIRenderer;

    // Bullet
    OgreBulletDynamics::DynamicsWorld *mWorld;
    OgreBulletCollisions::DebugDrawer *debugDrawer;
    int mNumEntitiesInstanced;
    std::deque<OgreBulletDynamics::RigidBody *> mBodies;
    std::deque<OgreBulletCollisions::CollisionShape *> mShapes;
    Vector3 gravityVector;
    AxisAlignedBox bounds;
    void createBox( Vector3 position, Vector3 velocity );

    // Audiere
    audiere::AudioDevicePtr device;
    audiere::OutputStreamPtr song;
    audiere::SoundEffectPtr boxSound;
    float get3DFactor( Vector3 listener, Vector3 source );

    // Camera Position
    float mCamHeight;
    float mCamHeightVelocity;
    void setCameraHeight( const FrameEvent& evt );

    // init Methods
    void initCamera(void);
    void initLight(void);
    void initTerrain(void);
    void initSky(void);
    void initFog(void);
    void initCursor(void);
    void initBullet(void);
    void initAudiere(void);

    // Misc
    bool bLMouseDown, bRMouseDown; // true if respective mouse button is down

public:
    GameApplication(void);
    virtual ~GameApplication(void);

protected:
    virtual void createScene(void);
    virtual void createFrameListener(void);
    virtual void destroyScene(void);
    virtual bool frameStarted( const FrameEvent& evt );
    virtual bool frameEnded( const FrameEvent& evt );
    virtual bool frameRenderingQueued( const FrameEvent& evt );
    virtual bool keyPressed( const OIS::KeyEvent& arg );
    virtual bool mousePressed( const OIS::MouseEvent& arg, OIS::MouseButtonID id);
	virtual bool mouseReleased( const OIS::MouseEvent& arg, OIS::MouseButtonID id);
    virtual bool mouseMoved( const OIS::MouseEvent& arg );
};

#endif // #ifndef __TutorialApplication_h_
