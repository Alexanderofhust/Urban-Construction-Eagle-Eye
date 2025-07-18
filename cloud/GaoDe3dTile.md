import * as THREE from 'three'
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js'
import { DRACOLoader } from 'three/examples/jsm/loaders/DRACOLoader.js'
import { TilesRenderer } from '3d-tiles-renderer'
import Layer from './Layer'

/**
 *  倾斜摄影3DTile图层
 *  class TilesLayer
 *  container: 容器
 *  map: 地图
 *  tilesURL: 数据文件路径,入口文件一般为 tileset.json
 *  zooms: 显示区间 [5,14]
 */
class TilesLayer extends Layer {
  // 切片原点位置 [lng,lat]
  #position = null

  // 切片渲染器集
  #tilesRendererArr = []

  // 数据来源地址
  #tilesURL = ''

  // 上一个被选中对象
  #lastPick = null

  constructor ({
    container, // 容器
    map,
    position,
    zooms,
    tilesURL
  }) {
    super(arguments[0])
    this.#tilesURL = tilesURL
    this.getData(position)
  }

  // 实例化构造完成，init后，会先执行这里
  onReady () {
    // 加载图层内容
    this.loadTiles()
  }

  getData (position) {
    // 地理坐标转three坐标系，不管用不用arr，都需要转换一个非空数组
    // 否则customCoords没实例化api会报错
    const res = this.customCoords.lngLatsToCoords(position)
    this.#position = res
  }

	loadTiles () {
    // 借助3d-tiles-renderer实现模型加载，都是网上扒下来的示例
    const dracoLoader = new DRACOLoader()
    dracoLoader.setDecoderPath(`${basePath}/static/three/examples/js/libs/gltf/`)
    const loader = new GLTFLoader()
    loader.setDRACOLoader(dracoLoader)

    const tilesRenderer = new TilesRenderer(this.#tilesURL)
    tilesRenderer.manager.addHandler(/\.gltf$/, loader)

    tilesRenderer.setCamera(this.camera)
    tilesRenderer.setResolutionFromRenderer(this.camera, this.renderer)

	  // 模型得手,放到scene
    this.scene.add(tilesRenderer.group)
    this.#tilesRendererArr.push(tilesRenderer)
  }

  // 该方法会在requestAnimationFrame中执行
  update () {
    for (const tilesRenderer of this.#tilesRendererArr) {
      // 更新模型渲染器
      tilesRenderer.update()
    }
  }

}




import * as THREE from 'three'

class Layer {
  // 图层显示范围
  #zooms = [3, 22]

  // 默认支持动画
  isAnimate = true

  // 图层中心坐标
  #center = null

  // 射线，用于做物体拾取
  #raycaster = null

  // 支持鼠标交互
  #interactAble = false

  constructor (conf) {
    this.container = conf.container
    this.map = conf.map
    this.#interactAble = conf.interact || false
    this.customCoords = this.map.customCoords
    this.layer = null
    // 显示范围
    if (conf.zooms) {
      this.#zooms = conf.zooms
    }
    // 支持动画
    if (conf.animate !== undefined) {
      this.isAnimate = conf.animate
    }
    // three相关属性
    this.camera = null
    this.renderer = null
    this.scene = null
    // 事件监听字典
    this.eventMap = {}
    this.initLayer()
  }

  // 初始化图层
  initLayer () {
    return new Promise((resolve) => {
      this.layer = new AMap.GLCustomLayer({
        zIndex: 9999,
        visible: this.isInZooms(),
        init: (gl) => {
          this.initThree(gl)
          this.onReady() 
          this.#center = this.customCoords.getCenter()
          this.animate()
          resolve()
        },
        render: () => {
          const { scene, renderer, camera, customCoords } = this

          // 重新定位中心，这样才能使当前图层与Loca图层共存时显示正常
          if (this.#center) {
            customCoords.setCenter(this.#center)
          }

          const { near, far, fov, up, lookAt, position } = customCoords.getCameraParams()

          camera.near = near// 近平面
          camera.far = far // 远平面
          camera.fov = fov // 视野范围
          camera.position.set(...position)
          camera.up.set(...up)
          camera.lookAt(...lookAt)

          // 更新相机坐标系
          camera.updateProjectionMatrix()

          renderer.render(scene, camera)

          // 这里必须执行！重新设置 three 的 gl 上下文状态
          renderer.resetState()
        }
      })

      this.map.add(this.layer)
    })
  }

  animate () {
    if (this.update) {
      this.update()
    }
    if (this.map) {
      this.map.render()
    }
    requestAnimationFrame(() => {
      this.animate()
    })
  }

}


const layer = new TilesLayer({
    container: container.value,
    map: getMap(),
    position: [113.55015, 22.761426], // 这个参数决定了模型的位置
    zooms: [4, 22], // 在哪个地图缩放等级可见
    zoom: 19.48,
    interact: false,// 是否做鼠标交互
    tilesURL: '${basePath}/tileset.json' // 入口文件地址
  })
