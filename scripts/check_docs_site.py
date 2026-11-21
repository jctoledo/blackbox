#!/usr/bin/env python3
"""Validate the rendered guide navigation, metadata and sitemap after build_docs.py."""
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlsplit
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1] / 'docs'
BASE = 'https://jctoledo.github.io/blackbox/'


class Page(HTMLParser):
    def __init__(self):
        super().__init__()
        self.links = []
        self.ids = set()
        self.h1 = 0
        self.canonical = []
        self.description = []
        self.title = False

    def handle_starttag(self, tag, attrs):
        attrs = dict(attrs)
        if attrs.get('id'):
            self.ids.add(attrs['id'])
        if tag == 'h1':
            self.h1 += 1
        if tag == 'title':
            self.title = True
        if tag == 'a' and attrs.get('href'):
            self.links.append(attrs['href'])
        if tag == 'link' and attrs.get('rel') == 'canonical':
            self.canonical.append(attrs.get('href'))
        if tag == 'meta' and attrs.get('name') == 'description':
            self.description.append(attrs.get('content', ''))


def main():
    paths = [ROOT / 'index.html', *sorted((ROOT / 'guides').glob('*.html'))]
    assert len(paths) > 1, 'Run build_docs.py first'
    pages = {}
    for path in paths:
        page = Page()
        page.feed(path.read_text())
        pages[path.resolve()] = page
        assert page.h1 == 1, f'{path}: expected one main heading'
        assert page.title and len(page.description) == 1 and page.description[0], path
        url = BASE + (str(path.relative_to(ROOT)) if path.name != 'index.html' else '')
        assert page.canonical == [url], f'{path}: wrong canonical'
    for path, page in pages.items():
        for href in page.links:
            parts = urlsplit(href)
            if parts.scheme or parts.netloc:
                continue
            target = (path.parent / unquote(parts.path)).resolve() if parts.path else path
            if target.is_dir():
                target /= 'index.html'
            assert target.is_relative_to(ROOT), f'{path}: link escapes published site: {href}'
            assert target.exists(), f'{path}: broken link: {href}'
            if parts.fragment and target in pages:
                assert unquote(parts.fragment) in pages[target].ids, f'{path}: missing anchor: {href}'
    urls = {node[0].text for node in ET.parse(ROOT / 'sitemap.xml').getroot()}
    for page in pages.values():
        assert page.canonical[0] in urls, f'Canonical missing from sitemap: {page.canonical}'
    print(f'Checked {len(pages)} pages: local links, anchors, headings, metadata and sitemap')


if __name__ == '__main__':
    main()
