"""Optional local browser checks; uses an already installed Playwright/browser.

No server or download. Example: python3 -m replay.check_browser --browser /path/to/chromium
This stays outside pytest's network-disabled ROS offline test suite.
"""
import argparse
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--browser', required=True, type=Path)
    parser.add_argument('--screenshots',type=Path,default=Path('/tmp/embodied-replay-browser'))
    args=parser.parse_args()
    from playwright.sync_api import sync_playwright
    args.screenshots.mkdir(parents=True,exist_ok=True)
    url=(Path(__file__).resolve().parents[1]/'web/replay/index.html').as_uri()
    checks=0
    with sync_playwright() as p:
        browser=p.chromium.launch(executable_path=str(args.browser),headless=True,
            args=['--no-sandbox','--disable-dev-shm-usage','--disable-background-networking'])
        for label,width,height in [('desktop',1440,1000),('mobile',390,844)]:
            context=browser.new_context(viewport={'width':width,'height':height},device_scale_factor=1)
            external=[]; errors=[]
            def block(route):
                if route.request.url.startswith(('http:','https:','ws:','wss:')):
                    external.append(route.request.url);route.abort()
                else:route.continue_()
            context.route('**/*',block)
            page=context.new_page();page.on('pageerror',lambda e:errors.append(str(e)))
            page.goto(url);page.locator('#summary .stat').first.wait_for()
            assert page.locator('#summary .stat').count()==4;checks+=1
            assert page.locator('#tasks .success').count()==4;checks+=1
            assert page.locator('#verdict').inner_text().startswith('运动实验 FAILED');checks+=1
            assert page.evaluate('document.documentElement.scrollWidth<=innerWidth');checks+=1
            page.screenshot(path=str(args.screenshots/(label+'.png')),full_page=True)
            page.locator('#timeline').evaluate('(e)=>{e.value=10;e.dispatchEvent(new Event("input"));}')
            assert '9.' in page.locator('#clock').inner_text() or '10.' in page.locator('#clock').inner_text();checks+=1
            page.locator('#play').click();page.wait_for_timeout(250);page.locator('#play').click()
            assert page.locator('#clock').inner_text()!='0.000 s';checks+=1
            page.locator('.event-jump').click()
            assert 'ROBOT NOT OBSERVED' in page.locator('#sample-state').inner_text();checks+=1
            assert '未观测' in page.locator('#telemetry').inner_text();checks+=1
            page.locator('#zoom').click();assert page.locator('#map').get_attribute('viewBox')=='14.7 -27.5 3.4 2.6';checks+=1
            page.locator('[data-experiment="D1"]').click();assert page.locator('#replay-name').inner_text().startswith('D1');checks+=1
            assert '不适用' in page.locator('#clearance-chart').text_content();checks+=1
            page.locator('[data-experiment="D2"]').click();assert '未形成近距交会' in page.locator('#verdict').inner_text();checks+=1
            assert page.evaluate('ReplayMath.frameAt([{t:0},{t:1},{t:2}],1.5)')==1;checks+=1
            assert page.evaluate('ReplayMath.path([{x:0,y:0,connect_previous:false},{x:1,y:1,connect_previous:false}],r=>r.x,r=>r.y)').count('M')==2;checks+=1
            assert len(errors)==0,errors;checks+=1
            assert len(external)==0,external;checks+=1
            context.close()
        browser.close()
    print(f'Browser checks: {checks} passed; desktop 1440×1000 + mobile 390×844; screenshots: {args.screenshots}')


if __name__=='__main__':
    main()
