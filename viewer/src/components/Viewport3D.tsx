import React, { useEffect, useRef, useState, useCallback } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader.js';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader.js';
import { PLYLoader } from 'three/examples/jsm/loaders/PLYLoader.js';
import type {
  SceneEntry,
  RepresentationType,
  ViewportConfig,
  BoundingBoxInfo,
  ActiveGeometryStats,
  CameraPresetAction,
} from '../types';
import { GeometryStatsDisplay } from './GeometryStatsDisplay';

interface Viewport3DProps {
  scene: SceneEntry;
  representation: RepresentationType;
  config: ViewportConfig;
  onLoadingChange: (loading: boolean, progress: number) => void;
  onBoundingBoxChange?: (info: BoundingBoxInfo) => void;
  onStatsChange?: (stats: ActiveGeometryStats) => void;
  syncCameraState?: {
    position: [number, number, number];
    target: [number, number, number];
  };
  onCameraUpdate?: (pos: [number, number, number], target: [number, number, number]) => void;
  customTitle?: string;
  onFallbackToPly?: () => void;
  presetAction?: CameraPresetAction;
}

export const Viewport3D: React.FC<Viewport3DProps> = ({
  scene,
  representation,
  config,
  onLoadingChange,
  onBoundingBoxChange,
  onStatsChange,
  syncCameraState,
  onCameraUpdate,
  customTitle,
  onFallbackToPly,
  presetAction,
}) => {
  const containerRef = useRef<HTMLDivElement>(null);
  const rendererRef = useRef<THREE.WebGLRenderer | null>(null);
  const sceneRef = useRef<THREE.Scene | null>(null);
  const cameraRef = useRef<THREE.PerspectiveCamera | null>(null);
  const controlsRef = useRef<OrbitControls | null>(null);
  const modelGroupRef = useRef<THREE.Group | null>(null);
  const gridHelperRef = useRef<THREE.GridHelper | null>(null);
  const axesHelperRef = useRef<THREE.AxesHelper | null>(null);

  const [errorMessage, setErrorMessage] = useState<string | null>(null);
  const [activeStats, setActiveStats] = useState<ActiveGeometryStats | null>(null);
  const modelDimRef = useRef<{ radius: number; fitDist: number }>({ radius: 3, fitDist: 5 });

  // Background color mapping
  const getBackgroundColor = (bg: string) => {
    switch (bg) {
      case 'black':
        return 0x000000;
      case 'charcoal':
        return 0x111827;
      case 'light':
        return 0xe2e8f0;
      case 'slate':
      default:
        return 0x0f172a;
    }
  };

  // Safe resource disposal helper
  const disposeObject = (obj: THREE.Object3D) => {
    obj.traverse((child) => {
      if ((child as THREE.Mesh).isMesh) {
        const mesh = child as THREE.Mesh;
        if (mesh.geometry) mesh.geometry.dispose();
        if (Array.isArray(mesh.material)) {
          mesh.material.forEach((mat) => {
            if ((mat as any).map) (mat as any).map.dispose();
            mat.dispose();
          });
        } else if (mesh.material) {
          if ((mesh.material as any).map) (mesh.material as any).map.dispose();
          mesh.material.dispose();
        }
      }
    });
  };

  // Initialize Three.js scene
  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;

    const width = container.clientWidth;
    const height = container.clientHeight;

    // Renderer
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setSize(width, height);
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.0;
    container.appendChild(renderer.domElement);
    rendererRef.current = renderer;

    // Scene
    const threeScene = new THREE.Scene();
    threeScene.background = new THREE.Color(getBackgroundColor(config.background));
    sceneRef.current = threeScene;

    // Camera
    const camera = new THREE.PerspectiveCamera(45, width / height, 0.05, 1000);
    camera.position.set(...scene.initial_camera.position);
    cameraRef.current = camera;

    // Controls
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.rotateSpeed = 0.8;
    controls.zoomSpeed = 1.0;
    controls.panSpeed = 0.8;
    controls.target.set(...scene.initial_camera.target);
    controlsRef.current = controls;

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 1.2);
    threeScene.add(ambientLight);

    const keyLight = new THREE.DirectionalLight(0xffffff, 1.5);
    keyLight.position.set(5, 10, 7);
    threeScene.add(keyLight);

    const fillLight = new THREE.DirectionalLight(0xffffff, 0.8);
    fillLight.position.set(-5, 5, -5);
    threeScene.add(fillLight);

    const hemiLight = new THREE.HemisphereLight(0xffffff, 0x334155, 0.6);
    threeScene.add(hemiLight);

    // Helpers
    const gridHelper = new THREE.GridHelper(10, 20, 0x38bdf8, 0x334155);
    gridHelper.position.y = -0.01;
    gridHelper.visible = config.showGrid;
    threeScene.add(gridHelper);
    gridHelperRef.current = gridHelper;

    const axesHelper = new THREE.AxesHelper(1.5);
    axesHelper.visible = config.showAxes;
    threeScene.add(axesHelper);
    axesHelperRef.current = axesHelper;

    // Model Group
    const modelGroup = new THREE.Group();
    threeScene.add(modelGroup);
    modelGroupRef.current = modelGroup;

    // Camera change listener for syncing
    controls.addEventListener('change', () => {
      if (onCameraUpdate && cameraRef.current && controlsRef.current) {
        const pos = cameraRef.current.position.toArray() as [number, number, number];
        const target = controlsRef.current.target.toArray() as [number, number, number];
        onCameraUpdate(pos, target);
      }
    });

    // Animation Loop
    let animationFrameId: number;
    const animate = () => {
      animationFrameId = requestAnimationFrame(animate);
      if (config.autoRotate && controlsRef.current) {
        controlsRef.current.autoRotate = true;
        controlsRef.current.autoRotateSpeed = 1.2;
      } else if (controlsRef.current) {
        controlsRef.current.autoRotate = false;
      }
      controls.update();
      renderer.render(threeScene, camera);
    };
    animate();

    // Resize Handler
    const handleResize = () => {
      if (!container || !renderer || !camera) return;
      const w = container.clientWidth;
      const h = container.clientHeight;
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.setSize(w, h);
    };
    window.addEventListener('resize', handleResize);

    return () => {
      cancelAnimationFrame(animationFrameId);
      window.removeEventListener('resize', handleResize);
      controls.dispose();
      renderer.dispose();
      if (container.contains(renderer.domElement)) {
        container.removeChild(renderer.domElement);
      }
    };
  }, []);

  // Handle Preset Camera Actions
  useEffect(() => {
    if (!presetAction || !cameraRef.current || !controlsRef.current) return;
    const camera = cameraRef.current;
    const controls = controlsRef.current;
    const d = modelDimRef.current.fitDist || 3.5;

    switch (presetAction.type) {
      case 'top':
        camera.position.set(0, d * 1.5, 0.001);
        controls.target.set(0, 0, 0);
        break;
      case 'front':
        camera.position.set(0, 0, d * 1.5);
        controls.target.set(0, 0, 0);
        break;
      case 'side':
        camera.position.set(d * 1.5, 0, 0);
        controls.target.set(0, 0, 0);
        break;
      case 'iso':
        camera.position.set(d * 0.8, d * 0.8, d * 0.8);
        controls.target.set(0, 0, 0);
        break;
      case 'reset':
      case 'overview':
        if (scene.initial_camera) {
          camera.position.set(...scene.initial_camera.position);
          controls.target.set(...scene.initial_camera.target);
        } else {
          camera.position.set(0, d * 0.75, d * 0.85);
          controls.target.set(0, 0, 0);
        }
        break;
      case 'facade':
        // Campus 0049 facade-focused view
        camera.position.set(0.55, 0.75, 0.95);
        controls.target.set(0.013, -0.079, 0.008);
        break;
      case 'roadway':
        // Forest Road roadway-focused downward view
        camera.position.set(0.021, 0.92, 0.82);
        controls.target.set(0.021, 0.089, -0.035);
        break;
      case 'bridge_structure':
        // River Bridge deck and piers structural view
        camera.position.set(1.27, 1.13, 0.52);
        controls.target.set(0.673, 0.232, -0.681);
        break;
      case 'vehicle_focus':
        // Desert Road Candidate B focus on preserved Mini Cooper at [0.03, 0.04, 0.45]
        camera.position.set(0.33, 0.28, 0.98);
        controls.target.set(0.084, -0.053, 0.536);
        break;
      case 'full_road':
        // Desert Road full corridor overview
        camera.position.set(0.0, 3.2, 1.8);
        controls.target.set(0.0, 0.0, -0.4);
        break;
      default:
        break;
    }
    controls.update();
  }, [presetAction, scene]);

  // Sync camera when external sync state changes
  useEffect(() => {
    if (syncCameraState && cameraRef.current && controlsRef.current) {
      cameraRef.current.position.set(...syncCameraState.position);
      controlsRef.current.target.set(...syncCameraState.target);
      controlsRef.current.update();
    }
  }, [syncCameraState]);

  // Update helpers and background on config change
  useEffect(() => {
    if (gridHelperRef.current) gridHelperRef.current.visible = config.showGrid;
    if (axesHelperRef.current) axesHelperRef.current.visible = config.showAxes;
    if (sceneRef.current) {
      sceneRef.current.background = new THREE.Color(getBackgroundColor(config.background));
    }
    // Update wireframe state on current model
    if (modelGroupRef.current) {
      modelGroupRef.current.traverse((child) => {
        if ((child as THREE.Mesh).isMesh) {
          const mesh = child as THREE.Mesh;
          if (Array.isArray(mesh.material)) {
            mesh.material.forEach((m) => {
              (m as any).wireframe = config.wireframe;
            });
          } else if (mesh.material) {
            (mesh.material as any).wireframe = config.wireframe;
          }
        }
      });
    }
  }, [config]);

  // Load Model Asset
  const loadModel = useCallback(() => {
    const modelGroup = modelGroupRef.current;
    const camera = cameraRef.current;
    const controls = controlsRef.current;
    const container = containerRef.current;
    if (!modelGroup || !camera || !controls) return;

    // Clear previous model
    while (modelGroup.children.length > 0) {
      const child = modelGroup.children[0];
      modelGroup.remove(child);
      disposeObject(child);
    }

    setErrorMessage(null);
    onLoadingChange(true, 0);

    const repConfig = scene.representations[representation] || scene.representations.textured_obj;
    const onProgress = (xhr: ProgressEvent) => {
      if (xhr.lengthComputable && xhr.total > 0) {
        const pct = Math.round((xhr.loaded / xhr.total) * 100);
        onLoadingChange(true, pct);
      }
    };

    const setupLoadedObject = (object: THREE.Object3D, format: 'OBJ' | 'PLY', sourceFileName: string) => {
      // Calculate Bounding Box and Bounding Sphere
      const box = new THREE.Box3().setFromObject(object);
      const center = box.getCenter(new THREE.Vector3());
      const size = box.getSize(new THREE.Vector3());
      const sphere = box.getBoundingSphere(new THREE.Sphere());
      const radius = sphere.radius || Math.max(size.x, size.y, size.z) / 2 || 1;

      // Report bounding box to parent
      if (onBoundingBoxChange) {
        onBoundingBoxChange({
          min: box.min.toArray() as [number, number, number],
          max: box.max.toArray() as [number, number, number],
          size: size.toArray() as [number, number, number],
          center: center.toArray() as [number, number, number],
        });
      }

      // Center container group so camera orbits around center without modifying original vertex coordinates!
      object.position.set(-center.x, -center.y, -center.z);

      // Apply initial wireframe state and double-sided material
      object.traverse((child) => {
        if ((child as THREE.Mesh).isMesh) {
          const mesh = child as THREE.Mesh;
          if (Array.isArray(mesh.material)) {
            mesh.material.forEach((m) => {
              (m as any).wireframe = config.wireframe;
              m.side = THREE.DoubleSide;
            });
          } else if (mesh.material) {
            (mesh.material as any).wireframe = config.wireframe;
            mesh.material.side = THREE.DoubleSide;
          }
        }
      });

      modelGroup.add(object);

      // Adaptive near/far clipping planes and framing distance
      const width = container?.clientWidth || 800;
      const height = container?.clientHeight || 600;
      const aspect = width / height;
      const fovRad = (camera.fov * Math.PI) / 180;
      const vDist = radius / Math.sin(fovRad / 2);
      const hDist = radius / Math.sin(Math.atan(Math.tan(fovRad / 2) * aspect));
      const fitDist = Math.max(vDist, hDist) * 1.15;

      modelDimRef.current = { radius, fitDist };

      camera.near = Math.max(0.01, fitDist / 150);
      camera.far = Math.max(1000, fitDist * 35);
      camera.updateProjectionMatrix();

      // Set initial camera framing
      if (scene.initial_camera) {
        camera.position.set(...scene.initial_camera.position);
        controls.target.set(...scene.initial_camera.target);
      } else {
        camera.position.set(0, fitDist * 0.75, fitDist * 0.85);
        controls.target.set(0, 0, 0);
      }
      controls.update();

      // Accurate Geometry Statistics
      let renderedPositions = 0;
      let triangleCount = 0;

      object.traverse((child) => {
        if ((child as THREE.Mesh).isMesh) {
          const geom = (child as THREE.Mesh).geometry;
          if (geom) {
            renderedPositions += geom.attributes.position?.count || 0;
            if (geom.index) {
              triangleCount += Math.round(geom.index.count / 3);
            } else {
              triangleCount += Math.round((geom.attributes.position?.count || 0) / 3);
            }
          }
        }
      });

      // Explicit distinction:
      // - Unique geometry vertices: source mesh unique vertices (from manifest/source file)
      // - Rendered position entries: actual position-buffer elements used by Three.js WebGL geometry
      // - Triangle count: rendered planar triangles
      const stats: ActiveGeometryStats = {
        uniqueVertices: scene.vertex_count || (format === 'PLY' ? renderedPositions : null),
        renderedPositions: renderedPositions,
        triangles: triangleCount,
        sourceFileName: sourceFileName,
        format: format,
      };

      setActiveStats(stats);
      if (onStatsChange) {
        onStatsChange(stats);
      }

      onLoadingChange(false, 100);
    };

    // Branch by loader type
    if (repConfig.type === 'obj' && repConfig.obj_url) {
      const mtlUrl = repConfig.mtl_url;
      const objUrl = repConfig.obj_url;
      const sourceFileName = objUrl.split('/').pop() || 'textured_mesh.obj';

      const objLoader = new OBJLoader();

      if (mtlUrl) {
        const mtlLoader = new MTLLoader();
        const basePath = mtlUrl.substring(0, mtlUrl.lastIndexOf('/') + 1);
        mtlLoader.setPath(basePath);

        const mtlFileName = mtlUrl.substring(mtlUrl.lastIndexOf('/') + 1);
        mtlLoader.load(
          mtlFileName,
          (materials) => {
            materials.preload();
            objLoader.setMaterials(materials);
            objLoader.load(
              objUrl,
              (obj) => setupLoadedObject(obj, 'OBJ', sourceFileName),
              onProgress,
              (err) => {
                console.error('OBJ load error:', err);
                setErrorMessage('Failed to load textured OBJ model');
                onLoadingChange(false, 0);
              }
            );
          },
          onProgress,
          (err) => {
            console.warn('MTL load error, falling back to untextured OBJ:', err);
            objLoader.load(
              objUrl,
              (obj) => setupLoadedObject(obj, 'OBJ', sourceFileName),
              onProgress,
              () => {
                setErrorMessage('Failed to load OBJ model');
                onLoadingChange(false, 0);
              }
            );
          }
        );
      } else {
        objLoader.load(
          objUrl,
          (obj) => setupLoadedObject(obj, 'OBJ', sourceFileName),
          onProgress,
          () => {
            setErrorMessage('Failed to load OBJ model');
            onLoadingChange(false, 0);
          }
        );
      }
    } else if (repConfig.type === 'ply' && repConfig.ply_url) {
      const plyUrl = repConfig.ply_url;
      const sourceFileName = plyUrl.split('/').pop() || 'surface_mesh.ply';
      const plyLoader = new PLYLoader();
      plyLoader.load(
        plyUrl,
        (geometry) => {
          geometry.computeVertexNormals();
          const hasColors = geometry.hasAttribute('color');
          const material = new THREE.MeshStandardMaterial({
            vertexColors: hasColors,
            roughness: 0.85,
            metalness: 0.1,
            color: hasColors ? 0xffffff : 0xcccccc,
            side: THREE.DoubleSide,
            wireframe: config.wireframe,
          });
          const mesh = new THREE.Mesh(geometry, material);
          setupLoadedObject(mesh, 'PLY', sourceFileName);
        },
        onProgress,
        (err) => {
          console.error('PLY load error:', err);
          setErrorMessage('Failed to load PLY model file');
          onLoadingChange(false, 0);
        }
      );
    } else {
      setErrorMessage('Selected representation format is not available');
      onLoadingChange(false, 0);
    }
  }, [scene, representation]);

  useEffect(() => {
    loadModel();
  }, [loadModel]);

  return (
    <div className="viewport-area" ref={containerRef}>
      {/* Floating Info Overlay */}
      <div className="viewport-floating-info">
        <div className="floating-title">
          {customTitle || scene.display_name}
        </div>
        <div style={{ fontSize: '0.72rem', color: 'var(--text-secondary)' }}>
          Representation: <span style={{ color: '#fff', fontWeight: 600 }}>{scene.representations[representation]?.label || representation}</span>
        </div>
        <GeometryStatsDisplay stats={activeStats} mode="overlay" />
      </div>

      {/* Error / Fallback Banner */}
      {errorMessage && (
        <div
          className="loading-overlay"
          style={{ background: 'rgba(15, 23, 42, 0.9)' }}
        >
          <div
            style={{
              color: 'var(--accent-rose)',
              fontWeight: 600,
              fontSize: '0.95rem',
            }}
          >
            {errorMessage}
          </div>
          {onFallbackToPly && representation === 'textured_obj' && (
            <button
              className="btn-action primary"
              onClick={onFallbackToPly}
              style={{ marginTop: '0.5rem' }}
            >
              Load PLY Representation Fallback
            </button>
          )}
        </div>
      )}
    </div>
  );
};
