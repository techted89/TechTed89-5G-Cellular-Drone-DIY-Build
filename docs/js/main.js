/**
 * TechTed89 5G Drone Hardware Guide — client-side enhancements
 * No build step required; pure vanilla JS.
 */

document.addEventListener('DOMContentLoaded', () => {
  // Set up anchor scroll offset for sticky header
  const header = document.querySelector('.site-header');
  const headerHeight = header ? header.offsetHeight : 72;
  document.querySelectorAll('a[href^="#"]').forEach((anchor) => {
    anchor.addEventListener('click', (e) => {
      const targetId = anchor.getAttribute('href');
      if (targetId === '#') return;
      const target = document.querySelector(targetId);
      if (target) {
        e.preventDefault();
        target.scrollIntoView({ behavior: 'smooth', block: 'start' });
      }
    });
  });

  // Smooth scroll for in-page anchors
  if (window.location.hash) {
    const hashTarget = document.querySelector(window.location.hash);
    if (hashTarget) {
      setTimeout(() => {
        hashTarget.scrollIntoView();
      }, 100);
    }
  }

  // Track which section is in view for scroll-spy
  const sections = document.querySelectorAll('section[id]');
  const navLinks = document.querySelectorAll('.nav a[href^="#"]');
  let scrollTimer;

  const scrollSpy = () => {
    clearTimeout(scrollTimer);
    scrollTimer = setTimeout(() => {
      const scrollPos = window.scrollY + 120;
      let current = null;
      for (const section of sections) {
        const top = section.offsetTop;
        const bottom = top + section.offsetHeight;
        if (scrollPos >= top && scrollPos < bottom) {
          current = section.getAttribute('id');
          break;
        }
      }
      navLinks.forEach((link) => {
        const href = link.getAttribute('href');
        if (href && href.startsWith('#')) {
          const id = href.slice(1);
          const active = id === current;
          link.style.color = active ? '#2563eb' : '#374151';
          link.fontWeight = active ? '700' : '400';
        }
      });
    }, 50);
  };

  window.addEventListener('scroll', scrollSpy, { passive: true });
  scrollSpy();

  // Table sorting by price (click column header not needed; keep simple)
  // Add live region announcements for accessibility
  const liveRegion = document.createElement('div');
  liveRegion.setAttribute('aria-live', 'polite');
  liveRegion.setAttribute('aria-atomic', 'true');
  liveRegion.className = 'sr-only';
  document.body.appendChild(liveRegion);

  // Update heading hierarchy / anchor targets check
  document.querySelectorAll('section[id]').forEach((s) => {
    const id = s.getAttribute('id');
    const heading = s.querySelector('h2');
    if (!heading) {
      console.warn(`Section ${id} has no h2 heading`);
    }
  });
});
