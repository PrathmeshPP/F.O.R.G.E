const { chromium } = require('C:/Users/25beevdt025/AppData/Local/OpenAI/Codex/runtimes/cua_node/78fb63cda1a9e2b2/bin/node_modules/playwright');
const path = require('path');
const fs = require('fs');

const OUT_DIR_1 = path.resolve(__dirname, '../outputs/accuracy_experiment/sprint8_interactive_viewer/screenshots');
const OUT_DIR_2 = path.resolve(__dirname, '../viewer/screenshots');

fs.mkdirSync(OUT_DIR_1, { recursive: true });
fs.mkdirSync(OUT_DIR_2, { recursive: true });

async function capture() {
  console.log('Launching Chromium for F.O.R.G.E Explorer visual validation...');
  const browser = await chromium.launch({
    headless: true,
    executablePath: 'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe',
    args: [
      '--enable-webgl',
      '--use-gl=angle',
      '--use-angle=default',
      '--ignore-gpu-blocklist',
      '--no-sandbox'
    ]
  });

  const page = await browser.newPage({
    viewport: { width: 1440, height: 900 }
  });

  page.on('console', msg => console.log('BROWSER LOG:', msg.text()));
  page.on('pageerror', err => console.error('BROWSER ERROR:', err.message));

  console.log('Navigating to http://127.0.0.1:5173/ ...');
  await page.goto('http://127.0.0.1:5173/', { waitUntil: 'networkidle', timeout: 30000 });
  await page.waitForTimeout(2000);

  const targets = [
    { name: 'screenshot_campus_0049.png', selector: '.scene-card:nth-child(1)', desc: 'Campus 0049' },
    { name: 'screenshot_desert_road_candidate_a.png', selector: '.scene-card:nth-child(2)', desc: 'Desert Road Cand A' },
    { name: 'screenshot_desert_road_candidate_b.png', selector: '.scene-card:nth-child(3)', desc: 'Desert Road Cand B' },
    { name: 'screenshot_forest_road.png', selector: '.scene-card:nth-child(4)', desc: 'Forest Road' },
    { name: 'screenshot_river_bridge.png', selector: '.scene-card:nth-child(5)', desc: 'River Bridge' },
  ];

  for (const t of targets) {
    console.log(`\nSelecting ${t.desc}...`);
    await page.click(t.selector);
    // Wait for model load spinner to disappear
    await page.waitForTimeout(3000);
    // Slight orbit interaction to demonstrate 3D responsiveness
    const canvas = await page.$('canvas');
    if (canvas) {
      const box = await canvas.boundingBox();
      if (box) {
        await page.mouse.move(box.x + box.width / 2, box.y + box.height / 2);
        await page.mouse.down();
        await page.mouse.move(box.x + box.width / 2 + 50, box.y + box.height / 2 - 30);
        await page.mouse.up();
        await page.waitForTimeout(1000);
      }
    }

    const path1 = path.join(OUT_DIR_1, t.name);
    const path2 = path.join(OUT_DIR_2, t.name);
    await page.screenshot({ path: path1, fullPage: false });
    fs.copyFileSync(path1, path2);
    console.log(`Saved screenshot: ${path1}`);
  }

  // 6. Capture Side-by-Side Desert Road Comparison
  console.log('\nActivating Side-by-Side Comparison Mode...');
  // Click compare button in left sidebar
  await page.click('.quick-compare-box button');
  await page.waitForTimeout(4000);

  const compName = 'screenshot_desert_road_comparison.png';
  const compPath1 = path.join(OUT_DIR_1, compName);
  const compPath2 = path.join(OUT_DIR_2, compName);
  await page.screenshot({ path: compPath1, fullPage: false });
  fs.copyFileSync(compPath1, compPath2);
  console.log(`Saved comparison screenshot: ${compPath1}`);

  await browser.close();
  console.log('\nAll application screenshots captured successfully!');
}

capture().catch(err => {
  console.error('Fatal capture error:', err);
  process.exit(1);
});
